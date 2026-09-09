import copy
import tempfile
import unittest
from pathlib import Path
from jetpilot_console.map_detail import (_write_hd_map_yaml, _lanes_from_hd_data,
                                         _read_hd_map, load_yaml)
from jetpilot_console.network_geometry import network


def lanes():
    return [dict(id=name,closed_loop=False,boundary_mode='paired',centerline_mode='auto',
                 left_bound=[[start,1],[end,1]],right_bound=[[start,-1],[end,-1]],
                 centerline=[[start,0],[end,0]],successor_ids=successors,default_successor_id=successors[0] if len(successors)>1 else '')
            for name,start,end,successors in [('a',0,2,['b','c']),('b',2,4,[]),('c',2,4,[])]]


class NetworkRoundTripTest(unittest.TestCase):
    def write(self,path,rows):
        _write_hd_map_yaml(path,{},rows,'a',path.with_suffix('.csv'),
                           {'sections':[{'id':'s','lane_id':'a','start_s_m':0,'end_s_m':2}]})

    def test_network_and_generated_candidates_round_trip_and_stale_removal(self):
        rows=lanes()
        for l in rows:
            l['network_raceline']=copy.deepcopy(l['centerline'])
            l['network_source_hash']=network.source_hash(rows,[])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.yaml';self.write(p,rows)
            raw=load_yaml(p);loaded=_lanes_from_hd_data(raw)
            self.assertEqual(loaded[0]['successor_ids'],['b','c'])
            self.assertTrue(loaded[0]['network_raceline'])
            self.assertEqual(network.source_hash(rows,[]),network.source_hash(raw['lanes'],[]))
            detail,_=_read_hd_map(p)
            self.assertEqual(detail['lanes'][0]['successor_ids'],['b','c'])
            loaded[1]['left_bound'][1][1]=1.1
            self.write(p,loaded)
            self.assertTrue(all('network_raceline' not in l for l in load_yaml(p)['lanes']))

    def test_invalid_network_does_not_overwrite_existing_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.yaml';rows=lanes();self.write(p,rows)
            previous=p.read_bytes();rows[0]['successor_ids']=['unknown']
            with self.assertRaises(ValueError):self.write(p,rows)
            self.assertEqual(p.read_bytes(),previous)

    def test_loading_legacy_map_produces_no_implicit_edges(self):
        rows=lanes()
        for l in rows:
            del l['successor_ids'];l['default_successor_id']=''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.yaml';self.write(p,rows)
            self.assertTrue(all(not l['successor_ids'] for l in _lanes_from_hd_data(load_yaml(p))))

    def test_save_api_generates_each_branch_and_older_client_preserves_connections(self):
        import struct
        from types import SimpleNamespace
        from jetpilot_console.map_detail import save_hd_map
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'course';folder.mkdir()
            config=SimpleNamespace(map_root=root)
            (folder/'vslam_landmarks.png').write_bytes(b'\x89PNG\r\n\x1a\n'+b'\0'*8+struct.pack('>II',200,200))
            (folder/'vslam_landmarks.yaml').write_text('image: vslam_landmarks.png\nresolution: 0.05\norigin: [0, 0, 0]\n')
            rows=lanes()
            for l in rows:
                l['drivable_left_bound']=[[-1,1],[5,1]]
                l['drivable_right_bound']=[[-1,-1],[5,-1]]
            path=folder/'course_hd_map.yaml';self.write(path,rows)
            result=save_hd_map(config,{'map_dir':str(folder),'primary_lane_id':'a','lanes':rows,
                                      'generate_network_racelines':True})
            self.assertTrue(all(l['network_raceline'] for l in result['hd_map']['lanes']))
            raw=load_yaml(path)
            self.assertEqual(len(raw['lanes']),3)
            old_client=copy.deepcopy(rows)
            for l in old_client:
                del l['successor_ids'];del l['default_successor_id']
            save_hd_map(config,{'map_dir':str(folder),'primary_lane_id':'a','lanes':old_client})
            self.assertEqual(load_yaml(path)['lanes'][0]['successor_ids'],['b','c'])

    def test_external_geometry_edit_hides_stale_network_racelines_in_ui(self):
        rows=lanes()
        for l in rows:
            l['network_raceline']=copy.deepcopy(l['centerline'])
            l['network_source_hash']=network.source_hash(rows,[])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'map.yaml';self.write(p,rows)
            p.write_text(p.read_text().replace('0, 1, 0.0','0, 1.2, 0.0'))
            detail,_=_read_hd_map(p)
            self.assertTrue(all(not l['network_raceline'] for l in detail['lanes']))
