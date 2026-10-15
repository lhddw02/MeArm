"""MeArm cubes: manual control, XYZ reaching, or physical pick and place."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import mujoco
import numpy as np
from arm_control import MeArmController
from cube_scene import CUBES, CUBE_SIZE_M, load_scene
from pick_place import PickPlaceTask


class CubeSceneController:
    """Two control modes sharing the same four real position actuators."""
    def __init__(self,model,data):
        self.model,self.data=model,data
        self.arm=MeArmController(model,data)
        self.mode='manual'
        self.target_mm=None
        self._start=None
        self._goal=None
        self._start_time=0.0
        self._move_seconds=2.0
        self._target_site=int(model.site('reach_target').id)
        self.model.site_rgba[self._target_site,3]=0

    def set_manual(self):
        """Stop automatic writes; native Control sliders own the servo targets."""
        self.mode='manual'
        self.model.site_rgba[self._target_site,3]=0

    def set_auto(self,target_mm,move_seconds=2.0,grip_deg=None):
        """Validate first, then blend from current servo commands to an IK pose."""
        duration=float(move_seconds)
        if not math.isfinite(duration) or duration<=0:
            raise ValueError('Move duration must be finite and greater than zero.')
        target=np.asarray(target_mm,dtype=float)
        if target.shape!=(3,) or not np.all(np.isfinite(target)):
            raise ValueError('Target must be three finite world coordinates in millimeters.')
        start=self.arm.target
        # The existing IK helper commits valid controls. Restore immediately so
        # actual motion follows the timed trajectory rather than a target jump.
        try:
            goal=self.arm.move_to(*target,grip_deg=grip_deg)
        finally:
            self.data.ctrl[self.arm.actuator_ids]=start
        self._start=start
        self._goal=goal
        self._start_time=float(self.data.time)
        self._move_seconds=duration
        self.target_mm=target.copy()
        self.model.site_pos[self._target_site]=target/1000
        self.model.site_rgba[self._target_site,3]=.65
        self.mode='auto'

    @property
    def finished(self):
        return self.mode=='auto' and self.data.time-self._start_time>=self._move_seconds-1e-12

    @property
    def error_mm(self):
        if self.mode!='auto' or self.target_mm is None: return math.inf
        return float(np.linalg.norm(self.arm.tcp_position_mm-self.target_mm))

    def update(self):
        if self.mode!='auto': return
        t=float(np.clip((self.data.time-self._start_time)/self._move_seconds,0,1))
        blend=t*t*t*(10+t*(-15+6*t))
        self.data.ctrl[self.arm.actuator_ids]=self._start+blend*(self._goal-self._start)


def cube_target_mm(model,data,name,hover_mm=30.0):
    """Use the cube's current simulated center, plus the requested vertical offset."""
    if name not in {cube['name'] for cube in CUBES}:
        raise ValueError('Cube must be red, green, blue or yellow.')
    hover=float(hover_mm)
    if not math.isfinite(hover): raise ValueError('Hover offset must be finite.')
    target=1000*data.xpos[int(model.body('cube_'+name).id)].copy()
    target[2]+=hover
    return target


def print_status(controller):
    from terminal_control import print_status as print_arm_status
    print('Mode:',controller.mode)
    print_arm_status(controller.arm)
    if controller.mode=='auto':
        print(f'Auto target (mm): {controller.target_mm.tolist()}; '
              f'tool error={controller.error_mm:.3f} mm; '
              f'trajectory={"complete" if controller.finished else "moving"}')


HELP="""Cube scene commands (type in this terminal):
  manual                  allow native Control sliders and joint commands
  auto 160 0 120          automatically reach world XYZ in millimeters
  cube red                reach 30 mm above the current red cube center
  pick red 180 -45 90      pick the red cube and place its center at XYZ (mm)
  cancel                  cancel a running pick-and-place and release control
  yaw 20                  in manual mode: change one servo target in degrees
  joints 20 65 -20 15     in manual mode: yaw, shoulder, forearm, jaw opening
  status                  show mode, angles and measured tool position
  cubes                   show current cube centers in millimeters
  help                    show commands
  quit                    close the viewer
"""


def handle_scene_command(controller,line,move_seconds=2.0,hover_mm=30.0,task=None,clearance_mm=35):
    words=line.split()
    if not words: return False
    command,args=words[0].lower(),words[1:]
    if command in ('quit','exit','q') and not args: return True
    if command in ('help','?') and not args:
        print(HELP); return False
    if task is not None and task.active and command not in ('manual','cancel','status','cubes'):
        raise ValueError('Pick-and-place is running. Type cancel or manual before starting another command.')
    if command=='manual' and not args:
        if task is not None and task.active: task.cancel()
        controller.set_manual()
    elif command=='cancel' and not args and task is not None:
        task.cancel()
    elif command=='pick' and len(args)==4 and task is not None:
        task.start(args[0],list(map(float,args[1:])),move_seconds,clearance_mm)
    elif command=='auto' and len(args)==3:
        controller.set_auto(list(map(float,args)),move_seconds)
        if task is not None and task.color is not None: task.reset()
    elif command=='cube' and len(args)==1:
        target=cube_target_mm(controller.model,controller.data,args[0].lower(),hover_mm)
        controller.set_auto(target,move_seconds)
        if task is not None and task.color is not None: task.reset()
    elif command=='status' and not args:
        pass
    elif command=='cubes' and not args:
        for cube in CUBES:
            center=cube_target_mm(controller.model,controller.data,cube['name'],0)
            print(f'{cube["name"]}: {np.round(center,2).tolist()} mm')
        return False
    elif command in ('yaw','shoulder','forearm','grip','joints','home'):
        if controller.mode!='manual':
            raise ValueError('Switch to manual before entering joint commands or using sliders.')
        from terminal_control import handle_command
        return handle_command(controller.arm,line)
    else:
        raise ValueError('Unknown command or wrong number of values. Type help.')
    print_status(controller)
    if task is not None and task.color is not None:
        print(f'Pick-and-place: {task.status}, phase={task.phase}; {task.error}')
    return False


def render_scene(model,data,path):
    from PIL import Image
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[.075,0,.075]
    camera.distance=.50
    camera.azimuth=125
    camera.elevation=-28
    option=mujoco.MjvOption(); option.geomgroup[3:]=0
    with mujoco.Renderer(model,height=960,width=1280) as renderer:
        renderer.update_scene(data,camera,scene_option=option)
        pixels=renderer.render()
    path=Path(path).resolve(); path.parent.mkdir(parents=True,exist_ok=True)
    Image.fromarray(pixels).save(path)
    print('Rendered',path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('manual','auto'),default='manual')
    parser.add_argument('--target-mm',type=float,nargs=3,metavar=('X','Y','Z'),help='Tool reach target, or destination cube CENTER when combined with --cube (world mm)')
    parser.add_argument('--cube',choices=[cube['name'] for cube in CUBES],help='Select a cube; combine with --target-mm or --place-mm to pick and place')
    parser.add_argument('--place-mm',type=float,nargs=3,metavar=('X','Y','Z'),help='Explicit destination cube CENTER; requires --mode auto --cube COLOR')
    parser.add_argument('--clearance-mm',type=float,default=35,help='Lift clearance above the grasp TCP during pick-and-place')
    parser.add_argument('--hover-mm',type=float,default=30,help='Height above cube center for --cube (default: 30 mm)')
    parser.add_argument('--move-seconds',type=float,default=2,help='Duration of the automatic joint-space trajectory')
    parser.add_argument('--joints',type=float,nargs=4,metavar=('YAW','SHOULDER','FOREARM','GRIP'),help='Initial manual joint targets in geometric degrees')
    parser.add_argument('--grip-deg',type=float,help='Jaw opening; auto defaults to 28 degrees, manual defaults closed')
    parser.add_argument('--interactive',action='store_true',help='Allow mode switching and commands in this terminal')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,help='Finite run duration: default 30 s for pick/place or 6 s for reaching')
    parser.add_argument('--tolerance-mm',type=float,default=3,help='Measured TCP error threshold for reporting arrival')
    parser.add_argument('--render',type=Path,help='Save a still after simulating --seconds; no viewer required')
    parser.add_argument('--report',type=Path,help='Save the finite-run status and current cube positions as JSON')
    args=parser.parse_args()
    pick_requested=args.cube is not None and (args.target_mm is not None or args.place_mm is not None)
    if args.seconds is None: args.seconds=30 if pick_requested else 6
    for label,value in (('move-seconds',args.move_seconds),('seconds',args.seconds),('tolerance-mm',args.tolerance_mm)):
        if not math.isfinite(value) or value<=0: parser.error(f'--{label} must be finite and greater than zero')
    if not math.isfinite(args.hover_mm): parser.error('--hover-mm must be finite')
    if not math.isfinite(args.clearance_mm) or args.clearance_mm<20:
        parser.error('--clearance-mm must be finite and at least 20')
    if args.place_mm is not None and (args.cube is None or args.target_mm is not None):
        parser.error('--place-mm requires --cube and cannot be combined with --target-mm')
    if args.mode=='auto' and args.target_mm is None and args.cube is None:
        parser.error('auto mode requires --target-mm X Y Z or --cube COLOR')
    if args.mode=='manual' and (args.target_mm is not None or args.cube is not None or args.place_mm is not None):
        parser.error('Use --mode auto with --target-mm, --place-mm or --cube')
    if args.mode=='auto' and args.joints is not None:
        parser.error('--joints sets the initial pose only in manual mode')
    if args.joints is not None and args.grip_deg is not None:
        parser.error('Supply the grip angle in --joints or --grip-deg, not both')
    if pick_requested and args.grip_deg is not None:
        parser.error('Pick-and-place controls jaw opening automatically; omit --grip-deg')
    if args.interactive and (args.headless or args.render):
        parser.error('--interactive requires the live viewer')
    model,data=load_scene(grasp_ready=True)
    controller=CubeSceneController(model,data)
    task=PickPlaceTask(controller)
    try:
        if args.joints is not None:
            controller.arm.set_joint_targets(*args.joints)
        elif args.mode=='manual' and args.grip_deg is not None:
            controller.arm.set_joint_targets(grip_deg=args.grip_deg)
        if pick_requested:
            destination=args.place_mm if args.place_mm is not None else args.target_mm
            task.start(args.cube,destination,args.move_seconds,args.clearance_mm)
        elif args.mode=='auto':
            goal=args.target_mm if args.target_mm is not None else cube_target_mm(model,data,args.cube,args.hover_mm)
            controller.set_auto(goal,args.move_seconds,grip_deg=28.0 if args.grip_deg is None else args.grip_deg)
    except ValueError as exc:
        parser.error(str(exc))
    print('Scene: four physical 20 x 20 x 20 mm cubes on an 80 mm platform.')
    print_status(controller)
    if task.active:
        print(f'Picking {task.color}; destination cube center (mm): {task.destination_mm.tolist()}')
        print('Grasp uses physical pad contacts and friction. No cube attachment or pose reset.')
    last_phase=task.phase
    def tick():
        nonlocal last_phase
        task.update()
        mujoco.mj_step(model,data)
        if task.phase!=last_phase:
            print(f'[{data.time:.2f}s] {task.phase}',flush=True)
            last_phase=task.phase
    if args.headless or args.render:
        for _ in range(math.ceil(args.seconds/model.opt.timestep)):
            tick()
        mujoco.mj_forward(model,data)
        print_status(controller)
        if args.render: render_scene(model,data,args.render)
    else:
        from mujoco import viewer as mjviewer
        commands=queue.Queue()
        if args.interactive:
            print(HELP)
            def read_commands():
                for line in sys.stdin: commands.put(line)
                commands.put('quit')
            threading.Thread(target=read_commands,daemon=True).start()
            print('scene> ',end='',flush=True)
        print('Manual mode: expand Control in the right panel (Shift+Tab); native sliders use radians.')
        print('Auto mode: the controller owns the actuator targets until you switch to manual.')
        arrived=False
        with mjviewer.launch_passive(model,data,show_right_ui=True) as viewer:
            with viewer.lock():
                viewer.opt.geomgroup[3:]=0
                viewer.cam.lookat[:]=[.075,0,.075]
                viewer.cam.distance=.50
                viewer.cam.azimuth=125
                viewer.cam.elevation=-28
            while viewer.is_running():
                start=time.perf_counter()
                quit_requested=False
                with viewer.lock():
                    while not commands.empty():
                        try:
                            quit_requested=handle_scene_command(controller,commands.get_nowait(),args.move_seconds,args.hover_mm,task,args.clearance_mm)
                            arrived=False
                        except ValueError as exc:
                            print(f'Command rejected: {exc}')
                        if quit_requested: break
                        print('scene> ',end='',flush=True)
                    if not quit_requested:
                        tick()
                if quit_requested: break
                viewer.sync()
                if task.status=='complete' and not arrived:
                    print(f'Cube placed and released: error {task.report()["placement_error_mm"]:.3f} mm',flush=True)
                    arrived=True
                elif task.status=='failed' and not arrived:
                    print(f'Pick-and-place failed: {task.error}',flush=True)
                    arrived=True
                elif task.color is None and controller.finished and controller.error_mm<=args.tolerance_mm and not arrived:
                    print(f'Reached target: tool error {controller.error_mm:.3f} mm',flush=True)
                    arrived=True
                time.sleep(max(0,model.opt.timestep-(time.perf_counter()-start)))
    cube_positions={cube['name']:cube_target_mm(model,data,cube['name'],0).tolist() for cube in CUBES}
    result=task.report() if task.color is not None else None
    report=dict(mode=controller.mode,simulation_seconds=float(data.time),cube_size_mm=1000*CUBE_SIZE_M,
                cube_centers_mm=cube_positions,tcp_mm=controller.arm.tcp_position_mm.tolist(),
                target_mm=(result['destination_mm'] if result else None if controller.target_mm is None else controller.target_mm.tolist()),
                error_mm=(result['placement_error_mm'] if result else None if controller.mode!='auto' else controller.error_mm),
                reached=(result['status']=='complete' if result else bool(controller.finished and controller.error_mm<=args.tolerance_mm)),
                task_type='pick_place' if result else 'reach',pick_place=result,
                warnings=[str(index) for index,warning in enumerate(data.warning) if warning.number])
    if args.report:
        path=args.report.resolve(); path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print('Report:',path)
    if result:
        print(f'Pick-and-place {task.status}: cube error {result["placement_error_mm"]:.3f} mm; released={result["released"]}')
        if task.error: print(task.error)
    elif controller.mode=='auto':
        print('Target reached.' if report['reached'] else 'Target not reached yet; measured error shown above.')
    return 1 if task.status=='failed' else 0


if __name__=='__main__': raise SystemExit(main())
