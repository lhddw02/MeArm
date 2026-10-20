"""Synchronize MuJoCo actuator commands with a calibrated USB Arduino MeArm.

Starts disarmed. Type 'arm' only after matching the supported real arm to HOME.
Use --dry-run for an emulator that never opens a hardware port.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
if __package__:
    from ._runtime import configure_dependencies
    from .arduino_sync import DryRunLink, LinkError, SerialLink, Synchronizer
    from .hardware_calibration import Calibration
else:
    from _runtime import configure_dependencies
    from arduino_sync import DryRunLink, LinkError, SerialLink, Synchronizer
    from hardware_calibration import Calibration
configure_dependencies(ROOT)


HARDWARE_HELP = """Synchronization commands:
  arm                    enable hardware at calibrated HOME (sim-to-hardware only)
  disarm                 detach servo signals; support the real arm first
  estop                  detach and latch; Arduino reset + bridge restart required
  status                 show simulation targets and last applied pulse commands
  help                   joint/XYZ and synchronization commands
  quit                   disarm and close (observer mode only disconnects)
Simulation-to-hardware: use native Control sliders or terminal joint/XYZ commands.
Hardware-to-simulation: joint commands are disabled; Arduino LOCAL control is master.
STATE is the last servo pulse command, not measured physical position.
"""


def list_ports():
    try:
        from serial.tools import list_ports as serial_ports
    except ImportError as exc:
        raise LinkError("Install hardware support: python -m pip install -r requirements-hardware.txt") from exc
    ports = sorted(serial_ports.comports(), key=lambda port: port.device)
    for port in ports:
        print(f"{port.device}: {port.description} [{port.hwid}]")
    if not ports:
        print("No serial ports found. Connect the Arduino with a USB data cable and check Device Manager.")


def load_model(calibration, scene):
    import mujoco
    import numpy as np
    if __package__:
        from .arm_control import ACTUATOR_NAMES, MeArmController
    else:
        from arm_control import ACTUATOR_NAMES, MeArmController
    if scene == "cubes":
        if __package__:
            from .cube_scene import load_scene
        else:
            from cube_scene import load_scene
        model, data = load_scene(grasp_ready=True)
    else:
        model = mujoco.MjModel.from_xml_path(str(ROOT / "mujoco/mearm.xml"))
        data = mujoco.MjData(model)
    for name, joint in zip(ACTUATOR_NAMES, calibration.joints):
        index = model.actuator(name).id
        offset = 90 if joint.name == "shoulder" else 0
        window = np.deg2rad([joint.model_deg[0] - offset, joint.model_deg[-1] - offset])
        original = model.actuator_ctrlrange[index].copy()
        window[0] = max(window[0], original[0])
        window[1] = min(window[1], original[1])
        home = math.radians(joint.home_deg - offset)
        if window[0] > window[1] or not window[0] - 1e-10 <= home <= window[1] + 1e-10:
            raise ValueError(f"Calibration HOME for {joint.name} is outside the {scene} model's limits.")
        model.actuator_ctrlrange[index] = window
        data.ctrl[index] = home
    arm = MeArmController(model, data)
    mujoco.mj_forward(model, data)
    return model, data, arm


def print_sync_status(sync):
    if __package__:
        from .terminal_control import print_status
    else:
        from terminal_control import print_status
    print_status(sync.arm_controller)
    print(f"Link: {'DRY RUN' if sync.link.is_dry_run else 'USB'}; direction={sync.direction}; "
          f"streaming={'ON' if sync.enabled else 'OFF'}; calibration={sync.calibration.profile_id}")
    if sync.state is not None:
        print(f"Arduino armed={sync.state.armed}; reason={sync.state.reason}; "
              f"applied pulse commands (us)={sync.state.pulses}")
        print("Applied command angles (deg): " + ", ".join(f"{v:.2f}" for v in sync.applied_degrees))
    if sync.fault:
        print("FAULT: " + sync.fault)


def handle_sync_command(sync, line):
    if __package__:
        from .terminal_control import HELP, handle_command
    else:
        from terminal_control import HELP, handle_command
    words = line.split()
    if not words:
        return False
    name = words[0].lower()
    if len(words) == 1:
        if name in ("quit", "exit", "q"):
            return True
        if name in ("help", "?"):
            print(HARDWARE_HELP)
            print(HELP)
            return False
        if name == "status":
            print_sync_status(sync)
            return False
        if name in ("arm", "disarm", "estop"):
            getattr(sync, name)()
            print_sync_status(sync)
            return False
    if sync.direction == "hardware-to-sim":
        raise ValueError("Arduino is the master in hardware-to-sim mode; simulation motion commands are disabled.")
    # Validate BOTH model and hardware range; terminal errors must not commit a bad goal.
    before = sync.arm_controller.target.copy()
    try:
        result = handle_command(sync.arm_controller, line)
        sync.calibration.to_pulses(sync.arm_controller.target_degrees)
        return result
    except ValueError:
        sync.arm_controller.data.ctrl[sync.arm_controller.actuator_ids] = before
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument("--port", help="Arduino USB serial port, e.g. COM3")
    connection.add_argument("--dry-run", action="store_true", help="In-memory Arduino emulator, no hardware access")
    connection.add_argument("--list-ports", action="store_true", help="List USB serial ports and exit")
    parser.add_argument("--calibration", type=Path, help="Measured JSON calibration; required with --port")
    parser.add_argument("--direction", choices=("sim-to-hardware", "hardware-to-sim"), default="sim-to-hardware")
    parser.add_argument("--scene", choices=("arm", "cubes"), default="arm")
    parser.add_argument("--hz", type=float, default=20, help="Wall-clock link update rate (5..50 Hz)")
    parser.add_argument("--headless", action="store_true", help="No viewer; dry-run validation only")
    parser.add_argument("--seconds", type=float, default=3, help="Duration of a headless dry run")
    parser.add_argument("--report", type=Path, help="Save command/status report on exit")
    args = parser.parse_args(argv)
    if args.list_ports:
        try:
            list_ports()
        except LinkError as exc:
            parser.exit(1, str(exc) + "\n")
        return
    if not args.port and not args.dry_run:
        parser.error("Choose --dry-run or --port COM3; --list-ports lists available ports.")
    if args.port and args.calibration is None:
        parser.error("--port requires --calibration with your measured JSON file.")
    if args.headless and not args.dry_run:
        parser.error("Live hardware requires the interactive viewer; --headless is for --dry-run only.")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive.")
    if not math.isfinite(args.hz) or not 5 <= args.hz <= 50:
        parser.error("--hz must be between 5 and 50.")
    calibration_path = args.calibration or ROOT / "config/hardware_calibration.example.json"
    try:
        calibration = Calibration.load(calibration_path)
        if args.hz < 3000 / calibration.watchdog_ms:
            raise ValueError("--hz must provide at least three updates per watchdog interval.")
        model, data, arm = load_model(calibration, args.scene)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    link = None
    sync = None
    try:
        if args.dry_run:
            link = DryRunLink(calibration)
            print("DRY RUN: no USB port is opened and no real servos are controlled.")
        else:
            print(f"Opening {args.port} at {calibration.baudrate} baud; Uno/Nano may reset. Servos start disarmed.")
            link = SerialLink.open(args.port, calibration)
            if args.direction == "sim-to-hardware" and link.source != "HOST":
                raise LinkError("Firmware source is LOCAL; select --direction hardware-to-sim or upload HOST firmware.")
            if args.direction == "hardware-to-sim" and link.source != "LOCAL":
                raise LinkError("Observer mode requires LOCAL firmware and a local controller; HOST firmware needs sim-to-hardware.")
            if args.direction == "sim-to-hardware" and link.firmware_armed:
                link.disarm()
                raise LinkError("Board was already armed. It has been disarmed; support it and restart the bridge.")
        sync = Synchronizer(arm, calibration, link, args.direction, hz=args.hz)
        sync.update()
        print(HARDWARE_HELP)
        print("Calibration HOME (deg): " + ", ".join(map(str, calibration.home_degrees)))
        print("For live ARM: first support the arm and match it to HOME. ARM resets slider targets to HOME.")
        if args.headless:
            # Explicitly virtual: exercises slew and streaming without a device or keyboard.
            if args.direction == "sim-to-hardware":
                sync.arm()
            import mujoco
            started = time.monotonic()
            while time.monotonic() - started < args.seconds:
                tick = time.monotonic()
                sync.update(tick)
                mujoco.mj_step(model, data)
                time.sleep(max(0, model.opt.timestep - (time.monotonic() - tick)))
        else:
            run_viewer(model, data, sync)
        print_sync_status(sync)
    except (LinkError, OSError, ValueError) as exc:
        parser.exit(1, "Synchronization stopped: " + str(exc) + "\n")
    except KeyboardInterrupt:
        print("\nClosing synchronization.")
    finally:
        if sync is not None:
            sync.close()
            if args.report:
                report = {
                    "dry_run": args.dry_run, "direction": args.direction,
                    "calibration_id": calibration.profile_id,
                    "feedback_type": "applied_servo_pulse_command_not_measured_angle",
                    "simulation_target_degrees": list(map(float, arm.target_degrees)),
                    "last_applied_command_degrees": list(sync.applied_degrees) if sync.applied_degrees else None,
                    "last_state": sync.state.__dict__ if sync.state else None,
                    "fault": sync.fault, "disarm_requested_on_close": args.direction == "sim-to-hardware",
                }
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        elif link is not None:
            link.close()


def run_viewer(model, data, sync):
    import mujoco
    from mujoco import viewer as mjviewer
    commands = queue.Queue()

    def read_commands():
        for line in sys.stdin:
            commands.put(line)
        commands.put("quit")

    threading.Thread(target=read_commands, daemon=True).start()
    print("Type commands in this terminal. The MuJoCo viewer continues running.")
    print("mearm-sync> ", end="", flush=True)
    with mjviewer.launch_passive(model, data, show_right_ui=True) as viewer:
        viewer.opt.geomgroup[3:] = 0
        while viewer.is_running():
            started = time.monotonic()
            stop = False
            with viewer.lock():
                while not commands.empty():
                    line = commands.get_nowait()
                    try:
                        stop = handle_sync_command(sync, line)
                    except (ValueError, LinkError) as exc:
                        print("Command rejected: " + str(exc))
                    if stop:
                        break
                    print("mearm-sync> ", end="", flush=True)
                if stop:
                    break
                # A native slider outside the calibrated envelope is also rejected;
                # update latches a fault and detaches rather than clipping silently.
                sync.update(started)
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
