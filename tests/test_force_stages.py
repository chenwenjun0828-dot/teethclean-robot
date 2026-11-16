"""Short experiments stop at their own endpoint and return without teleporting."""
from dataclasses import asdict
from pathlib import Path
import tempfile
import unittest
import numpy as np

from control.force.compliance import ForceControlConfig,smoothstep
from control.force.contact_path import ContactPathConfig
from control.force.continuous_path import RegionalContactPath
from planning.config import Config,SweepOptions
from planning.test_sweep import straight_geometry,build_reference
from planning.infrastructure.trajectory import Trajectory
from planning.infrastructure.io import dump,fingerprint


class ForceStageTests(unittest.TestCase):
    def run_stage(self,stage,hold_target=None,line_trace=None):
        cfg,options=Config(),SweepOptions()
        reference=build_reference(straight_geometry(),cfg,options,1.)
        parameters=np.unique([p[k] for p in reference.parts for k in ('start','end')])
        count=len(parameters)+2
        plan=Trajectory(np.arange(float(count)),np.zeros((count,6)),
            ['transfer']+['trace']*(len(parameters)-1)+['retreat'],np.ones(count-1,int),list('abcdef'))
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            dump(folder/'sweep_reference.json',reference.record)
            report=dict(planning_mode='simple',config=asdict(cfg),sweep_options=asdict(options),
                sweep_reference_file=fingerprint(folder/'sweep_reference.json'),motion_segments=[dict(
                    status='planned',trace_start_waypoint=1,trace_end_waypoint=count-2,parameters=parameters.tolist())])
            path=RegionalContactPath(folder,plan,report,np.eye(4),ForceControlConfig(),ContactPathConfig(),
                                     stage=stage,hold_target=hold_target,line_trace=line_trace)
            states=[]
            for i in range(30000):
                force=(smoothstep(path.elapsed/.4) if path.stage=='acquire' else 1. if path.stage=='trace' else 0.)
                state=path.update(force,0.,.001,total_force_N=force)
                self.assertFalse(state['fault_reason'])
                states.append(state)
                if path.completed:break
            self.assertTrue(path.completed)
            return path,states

    def test_hold_does_not_advance_curve_and_returns_to_start(self):
        path,states=self.run_stage('hold')
        trace=[s for s in states if s['phase']=='trace']
        self.assertGreaterEqual(len(trace),5000)
        np.testing.assert_allclose([s['source_time_s'] for s in trace],1.)
        self.assertEqual(states[-1]['source_time_s'],0.)

    def test_short_is_five_mm_prefix_with_release_at_same_endpoint(self):
        path,states=self.run_stage('short')
        trace=[s for s in states if s['phase']=='trace']
        release=[s for s in states if s['phase']=='release']
        self.assertGreater(len(trace),0)
        self.assertAlmostEqual(trace[-1]['reference_parameter_m'],release[0]['reference_parameter_m'])
        self.assertLessEqual(max(s['source_time_s'] for s in trace),path.stage_stop_time+1e-12)
        self.assertEqual(states[-1]['source_time_s'],0.)
        self.assertLess(np.max(abs(np.diff([s['source_time_s'] for s in states]))),.02)

    def test_alternate_hold_moves_out_and_back_at_clearance(self):
        target=dict(normal_scan=[0.,0.,1.],target_q_rad=[.01]*6,
                    first_clearance_q_rad=[0.]*6,target_clearance_q_rad=[.01]*6)
        path,states=self.run_stage('hold',target)
        trace=[s for s in states if s['phase']=='trace']
        self.assertGreaterEqual(len(trace),5000)
        np.testing.assert_allclose(trace[0]['q'],target['target_q_rad'])
        transfers=[s for s in states if s['phase']=='transfer']
        self.assertTrue(any(0.<s['q'][0]<.01 for s in transfers))
        self.assertLess(np.max(abs(np.diff([s['q'][0] for s in states]))),.001)
        np.testing.assert_allclose(states[-1]['q'],np.zeros(6))

        path._transition('acquire')
        feedback_q, feedback_normal=path.feedback_reference()
        np.testing.assert_allclose(feedback_q,target['target_q_rad'])
        np.testing.assert_allclose(feedback_normal,target['normal_scan'])

    def test_local_short_advances_two_mm_then_retraces_at_clearance(self):
        target=dict(normal_scan=[0.,0.,1.],target_q_rad=[.01]*6,
                    first_clearance_q_rad=[0.]*6,target_clearance_q_rad=[.01]*6)
        line=dict(arc_mm=[0.,1.,2.],q_rad=[[.01]*6,[.015]*6,[.02]*6],
                  clearance_q_rad=[[.01]*6,[.015]*6,[.02]*6],
                  normal_scan=[[0.,0.,1.]]*3)
        path,states=self.run_stage('short',target,line)
        trace=[s for s in states if s['phase']=='trace']
        self.assertGreater(len(trace),0)
        self.assertAlmostEqual(trace[-1]['reference_parameter_m'],.002,places=5)
        np.testing.assert_allclose(trace[-1]['q'],[.02]*6,atol=1e-5)
        self.assertEqual(states[-1]['source_time_s'],0.)
