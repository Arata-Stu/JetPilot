import json
import csv
from unittest.mock import patch
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/app/backend'))
sys.path.insert(0,str(ROOT/'python_ws/jetpilot_e2e_training/src'))
from jetpilot_console import section_multihead_pipeline as pipeline
from e2e_learning.data.sections import map_digest
from test_sections import document


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.config = SimpleNamespace(python_ws=root/'python_ws',repo_root=ROOT, map_root=root/'maps',
            record_root=root/'bags', state_dir=root/'state',ros2_ws=root/'ros2_ws',python_bin='python3',jetson_user='jetson',jetson_ips=[])
        self.training = self.config.python_ws/'jetpilot_e2e_training'
        self.training.mkdir(parents=True)
        (self.training/'src').symlink_to(ROOT/'python_ws/jetpilot_e2e_training/src',target_is_directory=True)
        self.config.map_root.mkdir(); self.config.record_root.mkdir()
        self.map = self.config.map_root/'track_hd_map.yaml'; self.map.write_text(json.dumps(document()))
        self.bag = self.config.record_root/'bag'; self.bag.mkdir()
        self.weight = self.training/'weights'/'dino.pth'; self.weight.parent.mkdir(); self.weight.write_bytes(b'weights')
        self.datasets = []
        for name,sections in [('straight',['straight']),('curve',['curve']),('generic',['straight','curve'])]:
            d = self.training/'datasets'/name; d.mkdir(parents=True)
            (d/'section_dataset.json').write_text(json.dumps({'kind':'section_dataset','map':str(self.map),
                'map_document':document(),'map_sha256':map_digest(document()),'sections':sections,'recommended_throttle':.2,'sample_count':10}))
            (d/'samples.csv').write_text('sequence_id,stamp,image_path,steering,throttle\na,1,a.jpg,0,.2\n')
            self.datasets.append(d)

    def request(self,spec):
        return json.loads(Path(spec.command[spec.command.index('--request-file')+1]).read_text())

    def test_dynamic_maps_and_sections(self):
        result = pipeline.snapshot(self.config)
        self.assertEqual(result['maps'][0]['sections'],['straight','curve'])
        self.assertEqual(len(result['datasets']),3)
        d=document(); d['sections'].append({'id':'extra','start_s_m':1,'end_s_m':2,'lane_id':'lane'})
        self.map.write_text(json.dumps(d))
        self.assertIn('extra',pipeline.scan_maps(self.config)[0]['sections'])

    def test_multi_bag_preprocessing_request_is_a_file(self):
        other = self.config.record_root/'bag2'; other.mkdir()
        spec = pipeline.build_preprocess(self.config, {'dataset_name':'new', 'map':str(self.map),
            'sections':['straight','curve'],'rosbags':[str(self.bag),str(other)]})
        request = self.request(spec)
        self.assertEqual(len(request['rosbags']),2)
        self.assertEqual(request['sections'],['straight','curve'])
        self.assertNotIn('filter_throttle',request)

    def test_paths_and_unknown_section_rejected(self):
        base = {'dataset_name':'new','map':str(self.map),'sections':['missing'],'rosbags':[str(self.bag)]}
        with self.assertRaises(ValueError): pipeline.build_preprocess(self.config,base)
        with self.assertRaises(ValueError): pipeline.build_preprocess(self.config,{**base,'sections':['curve'],'dataset_name':'../escape'})
        with self.assertRaises(ValueError): pipeline.build_preprocess(self.config,{**base,'sections':['curve'],'rosbags':['/tmp']})

    def heads(self):
        return [{'name':d.name,'dataset':str(d),'generic':d.name=='generic','throttle':.2} for d in self.datasets]

    def test_training_assigns_all_datasets_and_preserves_policy(self):
        spec = pipeline.build_train(self.config,{'run_name':'three','backbone_weights':str(self.weight),'heads':self.heads()})
        request = self.request(spec)
        self.assertEqual(len(request['heads']),3)
        self.assertEqual(request['heads'][2]['sections'],['straight','curve'])
        self.assertEqual(request['epochs'],30)

    def test_non_finite_values_and_partial_generic_rejected(self):
        with self.assertRaises(ValueError): pipeline.number(float('inf'),'throttle',0,1)
        heads=self.heads(); heads[2]['dataset']=str(self.datasets[0])
        with self.assertRaises(ValueError): pipeline.build_train(self.config,{'run_name':'bad','backbone_weights':str(self.weight),'heads':heads})

    def test_dataset_map_revision_mismatch_rejected(self):
        path=self.datasets[0]/'section_dataset.json'; data=json.loads(path.read_text());data['map_document']['frame_id']='other';path.write_text(json.dumps(data))
        with self.assertRaises(ValueError): pipeline.build_train(self.config,{'run_name':'bad','backbone_weights':str(self.weight),'heads':self.heads()})

    def test_preprocess_merges_bags_and_keeps_only_selected_sections(self):
        from e2e_learning.cli import section_pipeline as worker
        from e2e_learning.data import bag_sections
        def extract(bag, directory, request):
            directory=Path(directory)
            (directory/'images').mkdir()
            for i in range(3): (directory/'images'/f'{i}.jpg').write_bytes(b'image')
            rows=[{'sequence_id':'old','stamp':str(i),'image_path':f'images/{i}.jpg','steering':'0.1','throttle':'0.2'} for i in range(3)]
            with (directory/'samples.csv').open('w',newline='') as fp:
                writer=csv.DictWriter(fp,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        output=Path(self.tmp.name)/'merged'
        request={'map_document':document(),'map':str(self.map),'output':str(output),'sections':['curve'],
                 'mode':'recorded','rosbags':['/bags/one','/bags/two'],'throttle':.2}
        with patch.object(worker,'extract_one',extract), patch.object(bag_sections,'recorded_labels',return_value={0:'straight',1:'curve',2:'unknown'}):
            worker.preprocess(request)
        rows=worker.read_rows(output)
        self.assertEqual(len(rows),2)
        self.assertEqual(len({r['sequence_id'] for r in rows}),2)
        self.assertTrue(all((output/r['image_path']).is_file() for r in rows))
        meta=json.loads((output/'section_dataset.json').read_text())
        self.assertEqual(meta['section_counts'],{'curve':2})
        self.assertEqual(meta['dropped'],{'straight':2,'unknown':2})

    def test_offline_requires_ros_and_saved_map(self):
        with self.assertRaises(ValueError): pipeline.build_preprocess(self.config,{'dataset_name':'offline','mode':'offline',
            'map':str(self.map),'sections':['straight'],'rosbags':[str(self.bag)]})

    def test_offline_job_sources_ros_and_locks_analysis_domain(self):
        directory=self.config.map_root/'track';directory.mkdir()
        hd=directory/'track_hd_map.yaml';hd.write_text(json.dumps(document()))
        (directory/'cuvslam_map').mkdir();(directory/'cuvslam_map'/'map.mdb').write_bytes(b'map')
        setup=self.config.ros2_ws/'install'/'setup.bash';setup.parent.mkdir(parents=True);setup.write_text('')
        self.config.analysis_ros_domain_id=122
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'0'}):
            spec=pipeline.build_preprocess(self.config,{'dataset_name':'offline','mode':'offline',
                'map':str(hd),'sections':['straight'],'rosbags':[str(self.bag)]})
        self.assertEqual(spec.command[:2],['bash','-lc'])
        self.assertIn(str(setup),spec.command[2])
        self.assertIn('analysis-ros-domain:122',spec.resource_keys)
        request=json.loads(next((self.config.state_dir/'section_requests').glob('*.json')).read_text())
        self.assertEqual(request['localization_map'],str(directory.resolve()))
        self.assertEqual(request['timestamp_source'],'header')

    def test_deploy_requires_verified_model(self):
        run=pipeline.roots(self.config)[1]/'model';run.mkdir(parents=True)
        (run/'metadata.json').write_text(json.dumps({'model_kind':'section_multihead','onnx_verified':False}))
        (run/'model.onnx').write_bytes(b'onnx')
        with self.assertRaises(ValueError): pipeline.build_deploy(self.config,{'run':str(run),'host':'jetson.local'})
        (run/'metadata.json').write_text(json.dumps({'model_kind':'section_multihead','onnx_verified':True}))
        spec=pipeline.build_deploy(self.config,{'run':str(run),'host':'jetson.local'})
        self.assertIn('--build-engine',spec.command)
        with self.assertRaises(ValueError): pipeline.build_deploy(self.config,{'run':str(run),'host':'jetson.local','remote_root':"/workspaces/models/x';touch /tmp/bad;#"})
        with self.assertRaises(ValueError): pipeline.build_deploy(self.config,{'run':str(run),'host':'host; touch bad'})


if __name__ == '__main__': unittest.main()
