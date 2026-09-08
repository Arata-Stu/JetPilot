import copy
import tempfile
import unittest
from pathlib import Path

import test_map_detail as fixtures
from jetpilot_console.map_environment import normalize_obstacles, obstacle_path_issue
from jetpilot_console.map_detail import (
    build_map_detail, save_hd_map, save_section_gates, load_yaml,
    _custom_line_geometry_validation, validate_raceline_environment,
)


class MapEnvironmentTest(unittest.TestCase):
    def obstacle(self):
        return {'id':'box', 'name':'段ボール', 'polygon':[[4.99,-.2],[5.01,-.2],[5.01,.2],[4.99,.2]], 'height_m':.3, 'margin_m':.1}

    def test_thin_obstacle_segment_crossing_and_margin(self):
        obstacles = normalize_obstacles([self.obstacle()])
        self.assertIn('box', obstacle_path_issue([[0,0],[10,0]], False, obstacles))
        self.assertIn('box', obstacle_path_issue([[0,.25],[10,.25]], False, obstacles))
        self.assertEqual(obstacle_path_issue([[0,.4],[10,.4]], False, obstacles), '')
        self.assertIn('box', obstacle_path_issue([[0,.4],[10,.4]], False, obstacles, .2))
        self.assertIn('box', obstacle_path_issue([[5,0]], False, obstacles))
        self.assertIn('box', obstacle_path_issue([[0,0],[0,2],[10,0]], True, obstacles))

    def test_invalid_obstacles_are_rejected(self):
        for patch in ({'polygon':[[0,0],[1,1],[0,1],[1,0]]}, {'height_m':float('nan')},
                      {'margin_m':-1}, {'polygon':[[0,0],[1,0],[0,0]]}):
            with self.assertRaises(ValueError):
                normalize_obstacles([{**self.obstacle(), **patch}])
        with self.assertRaises(ValueError):
            normalize_obstacles([self.obstacle(),self.obstacle()])

    def test_separate_corridors_obstacle_save_reload_and_topology_save(self):
        with tempfile.TemporaryDirectory() as td:
            config, folder = fixtures.CustomLineTest()._make_section_map(Path(td))
            detail = build_map_detail(config,str(folder))
            lane = detail['hd_map']['lanes'][0]
            # Physical corridor is unchanged while generation corridor narrows.
            lane['left_bound'] = [[0,.5],[5,.5],[10,.5]]
            lane['right_bound'] = [[0,-.5],[5,-.5],[10,-.5]]
            result = save_hd_map(config, {'map_dir':str(folder),'lanes':[lane], 'obstacles':[self.obstacle()]})
            self.assertEqual(result['hd_map']['lanes'][0]['drivable_left_bound'], [[0,1],[5,1],[10,1]])
            self.assertTrue(any('box' in issue for issue in result['environment_issues']))
            data = load_yaml(folder/f'{folder.name}_hd_map.yaml')
            self.assertEqual(data['obstacles'][0]['height_m'], .3)
            csv = (folder/f'{folder.name}_hd_map_centerline.csv').read_text()
            self.assertIn('0.500000,0.500000',csv)
            # Custom lines may use physical space outside the generation band.
            line = [{'x_m':0,'y_m':.8},{'x_m':10,'y_m':.8}]
            self.assertTrue(_custom_line_geometry_validation(folder,line,False)['valid'])
            collision = _custom_line_geometry_validation(folder,[{'x_m':0,'y_m':0},{'x_m':10,'y_m':0}],False)
            self.assertFalse(collision['valid'])
            with self.assertRaisesRegex(ValueError,'box'):
                validate_raceline_environment(folder/f'{folder.name}_hd_map.yaml', [[0,0],[10,0]], .1)
            validate_raceline_environment(folder/f'{folder.name}_hd_map.yaml', [[0,.6],[10,.6]], .1)
            saved = save_section_gates(config, {'map_dir':str(folder),'section_gates':result['hd_map']['section_gates']})
            self.assertEqual(saved['hd_map']['obstacles'], result['hd_map']['obstacles'])
            self.assertEqual(saved['hd_map']['lanes'][0]['drivable_left_bound'], lane['drivable_left_bound'])
            # Older clients must not erase separately authored physical bounds.
            legacy = copy.deepcopy(lane)
            legacy.pop('drivable_left_bound'); legacy.pop('drivable_right_bound')
            saved = save_hd_map(config, {'map_dir':str(folder),'lanes':[legacy]})
            self.assertEqual(saved['hd_map']['obstacles'], result['hd_map']['obstacles'])
            self.assertEqual(saved['hd_map']['lanes'][0]['drivable_left_bound'], lane['drivable_left_bound'])

    def test_failed_save_does_not_overwrite_environment(self):
        with tempfile.TemporaryDirectory() as td:
            config, folder = fixtures.CustomLineTest()._make_section_map(Path(td))
            path = folder/f'{folder.name}_hd_map.yaml'
            original=path.read_bytes()
            lane=build_map_detail(config,str(folder))['hd_map']['lanes'][0]
            lane['left_bound']=[[0,2],[5,2],[10,2]]
            with self.assertRaisesRegex(ValueError,'generation bounds'):
                save_hd_map(config,{'map_dir':str(folder),'lanes':[lane]})
            self.assertEqual(path.read_bytes(),original)


if __name__ == '__main__': unittest.main()
