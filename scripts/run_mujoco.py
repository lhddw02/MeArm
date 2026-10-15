"""Launch MeArm, run a bounded four-servo demo, or render a still image."""
from pathlib import Path
import sys, argparse, math, time, queue, threading
ROOT=Path(__file__).resolve().parents[1]
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
import numpy as np


def demo_control(t):
    return np.array([.40*math.sin(.55*t),.32*math.sin(.7*t),.35*math.sin(.7*t+1),.25+.22*math.sin(1.1*t)])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--render',type=Path)
    parser.add_argument('--gif',type=Path,help='Render a short physics-driven motion preview')
    pose=parser.add_mutually_exclusive_group()
    pose.add_argument('--joints',type=float,nargs=4,metavar=('YAW','SHOULDER','FOREARM','GRIP'),help='Initial geometric angles in degrees; neutral: 0 90 0 0')
    pose.add_argument('--target-mm',type=float,nargs=3,metavar=('X','Y','Z'),help='Initial tool target in world millimeters: X forward, Y left, Z up')
    parser.add_argument('--interactive',action='store_true',help='Accept live joint and XYZ commands in this terminal')
    args=parser.parse_args()
    if args.demo and (args.joints is not None or args.target_mm is not None or args.interactive):
        parser.error('--demo overwrites manual commands; choose demo or manual control')
    if args.interactive and (args.headless or args.render or args.gif):
        parser.error('--interactive is for the live viewer; omit --headless, --render and --gif')
    if args.gif and (args.joints is not None or args.target_mm is not None):
        parser.error('--gif renders the demo; use --render for a specified pose')
    if not math.isfinite(args.seconds) or args.seconds<0:
        parser.error('--seconds must be finite and nonnegative')
    m=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco/mearm.xml')); d=mujoco.MjData(m)
    from arm_control import MeArmController
    from terminal_control import print_status
    arm=MeArmController(m,d)
    try:
        if args.joints is not None:
            arm.set_joint_targets(**dict(zip(('yaw_deg','shoulder_deg','forearm_deg','grip_deg'),args.joints)))
        elif args.target_mm is not None:
            arm.move_to(*args.target_mm)
    except ValueError as exc:
        parser.error(str(exc))
    mujoco.mj_forward(m,d)
    if args.gif:
        from PIL import Image
        cam=mujoco.MjvCamera(); cam.lookat[:]=[.035,0,.07]; cam.distance=.42; cam.azimuth=125; cam.elevation=-22
        opt=mujoco.MjvOption(); opt.geomgroup[3:]=0
        frames=[]
        with mujoco.Renderer(m,height=480,width=640) as renderer:
            for _ in range(60):
                for _ in range(100):
                    d.ctrl[:]=demo_control(d.time); mujoco.mj_step(m,d)
                renderer.update_scene(d,cam,scene_option=opt)
                frames.append(Image.fromarray(renderer.render()))
        target=args.gif.resolve(); target.parent.mkdir(parents=True,exist_ok=True)
        frames[0].save(target,save_all=True,append_images=frames[1:],duration=100,loop=0)
        print('Rendered physics animation',target); return
    if args.render:
        from PIL import Image
        # A representative pose, with its constrained coordinates initialized
        # exactly before rendering. This is a visual, not a dynamics result.
        if args.joints is not None or args.target_mm is not None:
            values=dict(zip(('yaw_joint','shoulder_joint','elbow_joint','left_finger_joint'),arm.target))
        else:
            values={'yaw_joint':-.25,'shoulder_joint':-.18,'elbow_joint':-.22,'left_finger_joint':.25}
        spec=__import__('json').loads((ROOT/'config/model_spec.json').read_text())
        for l in spec['links']:
            j=l['joint']
            if not j: continue
            if j['mimic']: values[j['name']]=values[j['mimic']['joint']]*j['mimic']['multiplier']
            d.qpos[m.joint(j['name']).qposadr[0]]=values[j['name']]
        mujoco.mj_forward(m,d)
        cam=mujoco.MjvCamera(); cam.lookat[:]=[.035,0,.07]; cam.distance=.39; cam.azimuth=125; cam.elevation=-22
        opt=mujoco.MjvOption(); opt.geomgroup[3:]=0
        with mujoco.Renderer(m,height=1080,width=1440) as renderer:
            renderer.update_scene(d,cam,scene_option=opt)
            pixels=renderer.render()
        target=args.render.resolve(); target.parent.mkdir(parents=True,exist_ok=True); Image.fromarray(pixels).save(target)
        print('Rendered',target); return
    if args.headless:
        steps=int(args.seconds/m.opt.timestep)
        for _ in range(steps):
            if args.demo: d.ctrl[:]=demo_control(d.time)
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d)
        print(f'Completed {d.time:.3f}s')
        print_status(arm)
        return
    from mujoco import viewer as mjviewer
    commands=queue.Queue()
    if args.interactive:
        from terminal_control import HELP
        print(HELP)
        print('Focus this terminal to type commands; the MuJoCo window keeps simulating.')
        def read_commands():
            for line in sys.stdin:
                commands.put(line)
            commands.put('quit')
        threading.Thread(target=read_commands,daemon=True).start()
    else:
        print('Expand Control in the right viewer panel to move the four actuator sliders.')
        print('Shift+Tab toggles the right panel. Sliders use radians; shoulder is an offset from 90 degrees.')
    print_status(arm)
    if args.interactive: print('mearm> ',end='',flush=True)
    with mjviewer.launch_passive(m,d,show_right_ui=True) as viewer:
        viewer.opt.geomgroup[3:]=0
        while viewer.is_running():
            start=time.perf_counter()
            if args.interactive:
                from terminal_control import handle_command
                quit_requested=False
                while not commands.empty():
                    line=commands.get_nowait()
                    try:
                        quit_requested=handle_command(arm,line)
                    except ValueError as exc:
                        print(f'Command rejected: {exc}')
                    if quit_requested: break
                    print('mearm> ',end='',flush=True)
                if quit_requested: break
            if args.demo: d.ctrl[:]=demo_control(d.time)
            mujoco.mj_step(m,d); viewer.sync()
            time.sleep(max(0,m.opt.timestep-(time.perf_counter()-start)))


if __name__=='__main__': main()
