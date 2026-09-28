import csv
import json
import math
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from jetpilot_console import map_detail as maps
from jetpilot_console import map_registration as registration
import test_map_detail


class RegistrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.config, self.source = test_map_detail.CustomLineTest()._make_section_map(self.root)
        self.target = self.root / "competition"
        self.target.mkdir()
        (self.target / "vslam_landmarks.yaml").write_text(
            'image: vslam_landmarks.png\nresolution: 0.1\norigin: [-10, -10, 0]\n')
        (self.target / "vslam_landmarks.png").write_bytes(
            b'\x89PNG\r\n\x1a\n' + b'\0'*8 + struct.pack('>II', 400, 400))
        (self.target / "cuvslam_map").mkdir()
        (self.target / "cuvslam_map" / "map.mdb").write_bytes(b"new venue landmarks")
        self.payload = dict(source_map_dir=str(self.source), map_dir=str(self.target),
                            x_m=3, y_m=-2, yaw_deg=90)

    def apply(self, **values):
        payload = {**self.payload, **values}
        preview = registration.preview_registration(self.config, payload)
        return registration.apply_registration(self.config, {**payload, "preview_token": preview["preview_token"]})

    def test_rigid_geometry_and_optimized_profile_preserved(self):
        path = self.source / f"{self.source.name}_hd_map.yaml"
        data = maps.load_yaml(path)
        data['lanes'][0]['drivable_left_bound'] = data['lanes'][0]['left_bound']
        data['lanes'][0]['drivable_right_bound'] = data['lanes'][0]['right_bound']
        data['junctions'] = [{'id':'signal', 'position':[5, 2, .4], 'branches':{}}]
        data['obstacles'] = [{'id':'box', 'polygon':[[20,20],[21,20],[21,21]], 'height_m':.5, 'margin_m':.1}]
        path.write_text(json.dumps(data))
        original = {p.relative_to(self.source): p.read_bytes() for p in self.source.rglob('*') if p.is_file()}
        preview = registration.preview_registration(self.config, self.payload)
        self.assertFalse((self.target / "competition_hd_map.yaml").exists())
        self.assertEqual(preview['hd_map']['lanes'][0]['centerline'][0], [3, -2, 0])
        result = self.apply()
        transformed = maps.load_yaml(self.target / 'competition_hd_map.yaml')
        self.assertEqual(transformed['sections'], data['sections'])
        self.assertAlmostEqual(transformed['junctions'][0]['position'][0], 1)
        self.assertAlmostEqual(transformed['junctions'][0]['position'][1], 3)
        self.assertEqual(transformed['junctions'][0]['position'][2], .4)
        self.assertEqual(transformed['obstacles'][0]['height_m'], .5)
        self.assertEqual(transformed['source_raster']['map_yaml'], str(self.target / 'vslam_landmarks.yaml'))
        rows = [r for r in csv.reader((self.target/'competition_raceline.csv').read_text().splitlines(),delimiter=';') if r and not r[0].startswith('#')]
        self.assertEqual(rows[1][0], '5')
        self.assertEqual(rows[1][4:], ['0','2.8','0'])
        self.assertAlmostEqual(float(rows[1][3]), math.pi/2)
        self.assertAlmostEqual(float(rows[1][1]), 3)
        self.assertAlmostEqual(float(rows[1][2]), 3)
        self.assertEqual((self.target/'cuvslam_map/map.mdb').read_bytes(), b'new venue landmarks')
        self.assertEqual(original, {p.relative_to(self.source):p.read_bytes() for p in self.source.rglob('*') if p.is_file()})
        self.assertTrue(Path(result['registration_backup']).is_dir())

    def test_custom_line_integrity_and_speeds(self):
        detail = maps.create_custom_line(self.config, {'map_dir':str(self.source), 'name':'Tuned',
            'base':'centerline', 'default_speed_mps':1.1, 'section_speeds_mps':{'section_a':1.4}})
        line = detail['custom_lines'][0]
        maps.activate_custom_line(self.config, {'map_dir':str(self.source), 'id':line['id']})
        before = (self.source / f'{self.source.name}_custom_line.csv').read_text().splitlines()
        result = self.apply(yaw_deg=37)
        self.assertTrue(result['custom_lines'][0]['valid'], result['custom_lines'][0]['issue'])
        self.assertEqual(maps._read_custom_lines(self.target)['active_id'], line['id'])
        after = (self.target/'competition_custom_line.csv').read_text().splitlines()
        self.assertEqual([r.split(';')[4:] for r in before], [r.split(';')[4:] for r in after])
        self.assertEqual(result['custom_lines'][0]['section_speeds_mps'], {'section_a':1.4})

    def test_stale_preview_rejected(self):
        preview = registration.preview_registration(self.config, self.payload)
        (self.source/f'{self.source.name}_raceline.csv').write_text('changed')
        with self.assertRaisesRegex(ValueError,'preview'):
            registration.apply_registration(self.config,{**self.payload,'preview_token':preview['preview_token']})
        self.assertFalse((self.target/'competition_hd_map.yaml').exists())

    def test_reapply_uses_source_and_backs_up_previous_alignment(self):
        self.apply()
        old = (self.target/'competition_hd_map.yaml').read_bytes()
        result = self.apply(x_m=10)
        self.assertEqual(maps.load_yaml(self.target/'competition_hd_map.yaml')['lanes'][0]['centerline'][0], [10,-2,0])
        self.assertEqual((Path(result['registration_backup'])/'competition_hd_map.yaml').read_bytes(),old)

    def test_rollback_on_install_failure(self):
        self.apply()
        before = registration._files(self.target)
        original_replace = registration.os.replace
        def fail_yaml(src, dest):
            if str(dest) == str(self.target/'competition_hd_map.yaml'):
                raise OSError('simulated disk error')
            original_replace(src,dest)
        with patch.object(registration.os, 'replace', side_effect=fail_yaml):
            with self.assertRaisesRegex(OSError,'simulated'):
                self.apply(x_m=25)
        self.assertEqual(registration._files(self.target),before)

    def test_nonfinite_same_map_and_symlink_rejected(self):
        for value in (float('nan'), float('inf'), 'bad'):
            with self.assertRaises(ValueError):
                registration.preview_registration(self.config,{**self.payload,'yaw_deg':value})
        with self.assertRaises(ValueError):
            registration.preview_registration(self.config,{**self.payload,'map_dir':str(self.source)})
        (self.target/'competition_raceline.csv').symlink_to(self.source/f'{self.source.name}_raceline.csv')
        with self.assertRaisesRegex(ValueError,'symlink'):
            self.apply()

    def test_round_trip_preserves_distance_z_and_topology(self):
        original = maps.load_yaml(self.source/f'{self.source.name}_hd_map.yaml')
        tf = {'x_m':3.,'y_m':-2.,'yaw_deg':37.}
        rotated = registration.transform_hd_map(original,tf)
        angle = math.radians(37)
        inverse = {'x_m':-3*math.cos(angle)+2*math.sin(angle),
                   'y_m':3*math.sin(angle)+2*math.cos(angle),'yaw_deg':-37}
        restored = registration.transform_hd_map(rotated,inverse)
        for a,b in zip(original['lanes'][0]['centerline'], restored['lanes'][0]['centerline']):
            for x,y in zip(a,b): self.assertAlmostEqual(x,y)
        self.assertEqual(original['sections'],restored['sections'])

    def test_path_rebinding_does_not_rename_lane_ids(self):
        metadata = {'id':self.source.name+'_fast',
                    'source_path':self.source.name+'_raceline.csv'}
        changed = registration._remap_paths(metadata,self.source,self.target)
        self.assertEqual(changed['id'],metadata['id'])
        self.assertEqual(changed['source_path'],'competition_raceline.csv')


if __name__ == '__main__':
    unittest.main()
