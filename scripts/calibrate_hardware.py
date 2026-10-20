"""Prepare and inspect MeArm Sync calibration files; never opens hardware."""
from __future__ import annotations

import argparse
from pathlib import Path

if __package__:
    from .hardware_calibration import Calibration
else:
    from hardware_calibration import Calibration


ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Prepare measured servo calibration and export its Arduino header. No hardware motion.",
        epilog="The example numbers are illustrative. Measure your own arm before setting calibrated=true.")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--init", type=Path, metavar="JSON", help="Create an uncalibrated example; refuses existing files.")
    action.add_argument("--check", type=Path, metavar="JSON", help="Validate a profile and print measured ranges.")
    action.add_argument("--export-firmware", type=Path, metavar="JSON", help="Export a profile to Arduino calibration.h.")
    parser.add_argument("--output", type=Path, metavar="HEADER", help="Header output path (only with --export-firmware).")
    args = parser.parse_args(argv)
    if args.output is not None and args.export_firmware is None:
        parser.error("--output requires --export-firmware.")

    try:
        if args.init is not None:
            example = ROOT / "config/hardware_calibration.example.json"
            contents = example.read_text(encoding="utf-8")
            args.init.parent.mkdir(parents=True, exist_ok=True)
            with args.init.open("x", encoding="utf-8", newline="\n") as target:
                target.write(contents)
            print(f"Created {args.init.resolve()}")
            print("ILLUSTRATIVE / UNCALIBRATED: measure all anchors and home positions before enabling hardware.")
            return 0

        path = args.check if args.check is not None else args.export_firmware
        calibration = Calibration.load(path)
        if args.export_firmware is not None:
            target = args.output or ROOT / "arduino/MeArmSync/calibration.h"
            if target.resolve() == path.resolve():
                raise ValueError("Header output must differ from the calibration JSON path.")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(calibration.firmware_header(), encoding="utf-8", newline="\n")
            print(f"Exported {target.resolve()}")
            print("Recompile and upload MeArmSync after every calibration change.")
        print(f"Profile {calibration.profile_id}; calibrated={str(calibration.calibrated).lower()}; "
              f"baud={calibration.baudrate}; watchdog={calibration.watchdog_ms} ms")
        for joint in calibration.joints:
            print(f"  {joint.name}: D{joint.pin}; {joint.min_deg:g}..{joint.max_deg:g} degrees; "
                  f"{joint.min_us}..{joint.max_us} us; home={joint.home_deg:g} degrees / "
                  f"{joint.home_us} us; rate<={joint.max_us_per_second:g} us/s")
        if not calibration.calibrated:
            print("ILLUSTRATIVE / UNCALIBRATED: hardware arming is disabled.")
        return 0
    except (OSError, ValueError) as exc:
        parser.exit(2, f"Calibration error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
