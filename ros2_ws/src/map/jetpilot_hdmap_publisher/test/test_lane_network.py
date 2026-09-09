"""Standard-library tests of graph identity, repeated splits and union clearance."""
import copy
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jetpilot_hdmap_publisher.lane_network import Tracker, validate, source_hash, smooth_candidate
from jetpilot_hdmap_publisher.drivable_guard import Environment


def lane(name, points, successors=(), default=''):
    return dict(id=name,closed_loop=False,centerline=points,
                left_bound=[[p[0],p[1]+1] for p in points],
                right_bound=[[p[0],p[1]-1] for p in points],
                successor_ids=list(successors),default_successor_id=default or (successors[0] if len(successors)>1 else ''))


def graph():
    return [lane('entry',[[0,0],[1,0],[2,0]],['a','b']),
            lane('a',[[2,0],[3,0],[4,.1],[5,0],[6,0]],['merge']),
            lane('b',[[2,0],[3,0],[4,-.1],[5,0],[6,0]],['merge']),
            lane('merge',[[6,0],[7,0],[8,0]],['c','d']),
            lane('c',[[8,0],[9,0],[10,0]],['exit']),
            lane('d',[[8,0],[9,0],[10,0]],['exit']),
            lane('exit',[[10,0],[11,0],[12,0]])]


class NetworkTest(unittest.TestCase):
    def test_repeated_fork_merge_and_overlap_preserve_lane_identity(self):
        t=Tracker(graph(),'entry'); t.set_choices({'entry':'a','merge':'d'})
        t.update((1,0)); t.update((2,0),max_step=2)
        self.assertEqual(t.current_lane_id,'a')
        t.update((4,-.1),max_step=3)  # Exactly on b, still follows a.
        self.assertEqual(t.current_lane_id,'a')
        t.update((6,0),max_step=3); self.assertEqual(t.current_lane_id,'merge')
        t.update((8,0),max_step=3); self.assertEqual(t.current_lane_id,'d')
        t.update((10,0),max_step=3); self.assertEqual(t.current_lane_id,'exit')

    def test_missing_default_is_rejected_before_running_and_default_is_used(self):
        lanes=graph();lanes[0]['default_successor_id']=''
        with self.assertRaisesRegex(ValueError,'デフォルト分岐'):Tracker(lanes,'entry')
        t=Tracker(graph(),'entry');t.update((1,0));t.update((2,0),max_step=2)
        self.assertEqual(t.current_lane_id,'a')

    def test_selection_can_change_before_commit_distance(self):
        t=Tracker(graph(),'entry');t.update((.1,0))
        self.assertEqual(t.selected_next,'')
        t.set_choices({'entry':'b'});t.update((1,0));t.update((2,0),max_step=2)
        self.assertEqual(t.current_lane_id,'b')

    def test_committed_choice_cannot_flip_mid_approach(self):
        t=Tracker(graph(),'entry'); t.set_choices({'entry':'a'}); t.update((1,0))
        t.set_choices({'entry':'b'}); t.update((2,0),max_step=2)
        self.assertEqual(t.current_lane_id,'a')

    def test_no_transition_before_end_even_on_successor(self):
        lanes=graph(); t=Tracker(lanes,'a'); t.update((4,0))
        self.assertEqual(t.current_lane_id,'a')
        with self.assertRaises(ValueError): t.update((10,0))
        self.assertEqual(t.current_lane_id,'a')

    def test_unknown_choices_and_edges_rejected(self):
        t=Tracker(graph(),'entry')
        with self.assertRaises(ValueError): t.set_choices({'entry':'exit'})
        for mutation in ('unknown','closed','gap','duplicate','heading'):
            lanes=graph()
            if mutation=='unknown': lanes[0]['successor_ids']=['missing']
            if mutation=='closed': lanes[0]['closed_loop']=True
            if mutation=='gap': lanes[1]['centerline'][0]=[2.1,0]
            if mutation=='duplicate': lanes[0]['successor_ids']=['a','a']
            if mutation=='heading': lanes[1]['centerline'][1]=[1,0]
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): validate(lanes)

    def test_next_fork_has_no_combinatorial_paths(self):
        lanes=graph(); t=Tracker(lanes,'entry');t.set_choices({'entry':'a','merge':'c'})
        points=t.path(horizon=100)
        self.assertEqual(points[-1],(12.,0.));self.assertLess(len(points),140)
        self.assertNotIn((4.,-.1),points)

    def test_explicit_cycle_has_bounded_lookahead(self):
        lanes=[]
        for i in range(4):
            points=[[3*math.cos((i+j/12)*math.pi/2),3*math.sin((i+j/12)*math.pi/2)] for j in range(13)]
            lanes.append(lane(str(i),points,[str((i+1)%4)]))
        t=Tracker(lanes,'0');points=t.path(horizon=100)
        self.assertLess(len(points),210)
        self.assertLess(math.dist(points[0],points[-1]),1e-6)

    def test_hash_survives_yaml_xyz_and_precision_but_rejects_edits(self):
        lanes=graph(); other=copy.deepcopy(lanes)
        for l in other:
            for key in ('left_bound','right_bound','centerline'):
                l[key]=[[float(p[0]),float(p[1]),0.] for p in l[key]]
        self.assertEqual(source_hash(lanes,[]),source_hash(other,[]))
        other[0]['centerline'][1][1]=.2
        self.assertNotEqual(source_hash(lanes,[]),source_hash(other,[]))

    def test_raceline_has_fixed_entry_exit_tangents_and_clearance(self):
        env=Environment([([[-1,2],[7,2]],[[-1,-2],[7,-2]],False)],[])
        l=graph()[1]; p=smooth_candidate(l,env,.16)
        self.assertEqual(p[0],[2.,0.]);self.assertEqual(p[-1],[6.,0.])
        self.assertAlmostEqual(p[1][1],0);self.assertAlmostEqual(p[-2][1],0)
        self.assertTrue(all(not env.issue(a,b,.16) for a,b in zip(p,p[1:])))
        self.assertLess(max(v[1] for v in p),.1)


class UnionTest(unittest.TestCase):
    def env(self,start=5., obstacles=()):
        return Environment([([(0,1),(5,1)],[(0,-1),(5,-1)],False),
                            ([(start,1),(10,1)],[(start,-1),(10,-1)],False)],obstacles)

    def test_exact_shared_seam_and_overlap_are_not_walls(self):
        for start in (4.,5.):
            self.assertEqual(self.env(start).issue((4.5,0),(5.5,0),.3),'')

    def test_identical_lanes_keep_outer_wall(self):
        env=Environment([([(0,1),(5,1)],[(0,-1),(5,-1)],False)]*2,[])
        self.assertIn('bounds',env.issue((2,.9),(3,.9),.2))

    def test_thin_gap_remains_forbidden(self):
        for gap in (.1,.000001):
            self.assertIn('bounds',self.env(5.+gap).issue((4,0),(6,0),.1))

    def test_obstacle_in_overlap_still_blocks(self):
        box=('box',[(4.9,-.1),(5.1,-.1),(5.1,.1),(4.9,.1)],0.)
        self.assertIn('box',self.env(4.,[box]).issue((4.5,0),(5.5,0),.2))

    def test_crossed_corridors_union_and_concave_corner(self):
        env=Environment([([(-3,1),(3,1)],[(-3,-1),(3,-1)],False),
                         ([(-1,-3),(-1,3)],[(1,-3),(1,3)],False)],[])
        self.assertEqual(env.issue((-.5,0),(.5,0),.4),'')
        self.assertIn('bounds',env.issue((.9,.9),(1.5,1.5),.2))


if __name__=='__main__': unittest.main()
