"""Exercise the live commands exactly as users type them into the terminal."""
from pathlib import Path
import contextlib
import io
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
import numpy as np
from arm_control import MeArmController
from terminal_control import handle_command


class TerminalControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco/mearm.xml'))

    def setUp(self):
        self.data=mujoco.MjData(self.model)
        mujoco.mj_forward(self.model,self.data)
        self.arm=MeArmController(self.model,self.data)

    def command(self,line):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            stopped=handle_command(self.arm,line)
        return stopped,out.getvalue()

    def test_individual_geometric_angles(self):
        for line in ('yaw 20','shoulder 65','forearm -20','grip 15'):
            stopped,_=self.command(line)
            self.assertFalse(stopped)
        np.testing.assert_allclose(self.arm.target,np.deg2rad([20,-25,-20,15]))

    def test_joint_command_matches_documented_example(self):
        _,text=self.command('joints 20 65 -20 15')
        np.testing.assert_allclose(self.arm.target,np.deg2rad([20,-25,-20,15]))
        self.assertIn('shoulder=65.00',text)

    def test_xyz_preserves_grip_and_accepts_optional_opening(self):
        self.command('grip 15')
        self.command('xyz 120 30 120')
        np.testing.assert_allclose(self.arm.forward_kinematics(),[120,30,120],atol=1e-8)
        self.assertAlmostEqual(self.arm.target[3],np.deg2rad(15))
        self.command('xyz 120 -30 120 10')
        np.testing.assert_allclose(self.arm.forward_kinematics(),[120,-30,120],atol=1e-8)
        self.assertAlmostEqual(self.arm.target[3],np.deg2rad(10))

    def test_bad_commands_do_not_partially_move(self):
        self.command('joints 20 65 -20 15')
        before=self.data.ctrl.copy()
        for line in ('joints 20 650 -20 15','xyz 1000 0 120','yaw nan','joints 1 2','unknown'):
            with self.assertRaises(ValueError): self.command(line)
            np.testing.assert_array_equal(self.data.ctrl,before)

    def test_home_and_quit(self):
        self.command('joints 20 65 -20 15')
        self.command('home')
        np.testing.assert_allclose(self.arm.target,[0,0,0,0],atol=1e-12)
        self.assertTrue(self.command('quit')[0])

    def test_native_slider_targets_survive_partial_commands(self):
        self.data.ctrl[self.model.actuator('shoulder').id]=np.deg2rad(10)
        self.command('yaw -20')
        self.assertAlmostEqual(self.arm.target[1],np.deg2rad(10))
        _,text=self.command('status')
        self.assertIn('shoulder=100.00',text)


if __name__=='__main__': unittest.main()
