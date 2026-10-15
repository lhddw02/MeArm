#!/usr/bin/env python3
"""Send geometric MeArm angles to the Gazebo Sim joint controllers."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = ROOT / "config" / "model_spec.json"


def joint_targets(yaw: float, shoulder: float, forearm: float, grip: float) -> dict[str, float]:
    """Angles are degrees; shoulder/forearm are elevations above horizontal."""
    return {
        "yaw_joint": math.radians(yaw),
        "shoulder_joint": math.radians(shoulder - 90.0),
        "elbow_joint": math.radians(forearm),
        "left_finger_joint": math.radians(grip),
    }


def command_pose(spec: dict, pose: tuple[float, float, float, float], *,
                 dry_run: bool = False, gz: str = "gz") -> None:
    joints = {link["joint"]["name"]: link["joint"] for link in spec["links"]
              if link.get("joint")}
    targets = joint_targets(*pose)
    # Validate the complete request before sending any part of it.
    for name, value in targets.items():
        joint = joints[name]
        if not math.isfinite(value) or not joint["lower"] - 1e-10 <= value <= joint["upper"] + 1e-10:
            raise ValueError(f"{name}: {math.degrees(value):.3f} degrees is outside "
                             f"[{math.degrees(joint['lower']):.3f}, {math.degrees(joint['upper']):.3f}]")
    for name, value in targets.items():
        args = [gz, "topic", "-t", f"/model/{spec['name']}/joint/{name}/0/cmd_pos",
                "-m", "gz.msgs.Double", "-p", f"data: {value:.12g}"]
        if dry_run:
            print(subprocess.list2cmdline(args))
        else:
            subprocess.run(args, check=True, timeout=10)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--yaw", type=float, default=0, help="base yaw in degrees")
    parser.add_argument("--shoulder", type=float, default=90,
                        help="upper-arm absolute elevation in degrees (neutral: 90)")
    parser.add_argument("--forearm", type=float, default=0,
                        help="forearm absolute elevation in degrees (neutral: 0)")
    parser.add_argument("--grip", type=float, default=0,
                        help="angle of each jaw in degrees (zero: closed)")
    parser.add_argument("--demo", action="store_true", help="run a short sequence, ending at neutral")
    parser.add_argument("--dry-run", action="store_true", help="print commands without needing Gazebo")
    parser.add_argument("--gz", default="gz", help="Gazebo command executable")
    args = parser.parse_args()
    if not args.dry_run and shutil.which(args.gz) is None:
        parser.error(f"Gazebo command {args.gz!r} is not on PATH; see docs/gazebo.md")
    try:
        spec = json.loads(args.spec.read_text(encoding="utf-8"))
        poses = [(args.yaw, args.shoulder, args.forearm, args.grip)]
        if args.demo:
            poses = [(0, 90, 0, 0), (25, 65, -20, 20),
                     (-25, 100, 25, 5), (0, 90, 0, 0)]
        for index, pose in enumerate(poses):
            print(f"Pose {index + 1}: yaw={pose[0]:g}, shoulder={pose[1]:g}, "
                  f"forearm={pose[2]:g}, grip={pose[3]:g} degrees", flush=True)
            command_pose(spec, pose, dry_run=args.dry_run, gz=args.gz)
            if args.demo and not args.dry_run and index < len(poses) - 1:
                time.sleep(2.5)
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as exc:
        print(f"Command failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
