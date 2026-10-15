"""Edit the steps below to program MeArm movements in degrees and millimeters."""
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
from mujoco import viewer as mjviewer
from arm_control import MeArmController


def main():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco/mearm.xml'))
    data=mujoco.MjData(model)
    arm=MeArmController(model,data)
    # Edit these operations; shoulder/forearm are absolute elevations.
    steps=[
        ('joint angles',lambda: arm.set_joint_targets(yaw_deg=20,shoulder_deg=65,forearm_deg=-20,grip_deg=15)),
        ('XYZ left',lambda: arm.move_to(120,30,120)),
        ('XYZ right',lambda: arm.move_to(120,-30,120)),
        ('close gripper',lambda: arm.set_joint_targets(grip_deg=0)),
        ('home',lambda: arm.set_joint_targets(yaw_deg=0,shoulder_deg=90,forearm_deg=0,grip_deg=0)),
    ]
    mujoco.mj_forward(model,data)
    with mjviewer.launch_passive(model,data) as viewer:
        viewer.opt.geomgroup[3:]=0
        for label,command in steps:
            if not viewer.is_running(): return
            command()
            print(label,flush=True)
            deadline=data.time+3
            while viewer.is_running() and data.time<deadline:
                start=time.perf_counter()
                mujoco.mj_step(model,data); viewer.sync()
                time.sleep(max(0,model.opt.timestep-(time.perf_counter()-start)))
        print('Sequence finished. The viewer remains open for actuator sliders.')
        while viewer.is_running():
            start=time.perf_counter()
            mujoco.mj_step(model,data); viewer.sync()
            time.sleep(max(0,model.opt.timestep-(time.perf_counter()-start)))


if __name__=='__main__': main()
