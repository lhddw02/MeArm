"""Create/reuse a Windows venv, install project dependencies, and launch Viser.

Uses only the Python standard library until the environment is ready. Normally
invoked by setup_windows.cmd; it also works with an explicit Python executable.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import venv

ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS = ("requirements-viser.txt", "requirements-hardware.txt")


def child_environment():
    """Do not inherit paths pointing at another computer's binary packages."""
    environment = os.environ.copy()
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    return environment


def run(command, *, root=ROOT):
    # Never build a shell command: paths and forwarded scene arguments may
    # contain spaces, parentheses, or other shell-significant characters.
    print("\n> " + subprocess.list2cmdline([str(value) for value in command]), flush=True)
    subprocess.run([str(value) for value in command], cwd=root,
                   env=child_environment(), check=True)


def check_bootstrap_python():
    if os.name != "nt":
        raise ValueError("This launcher targets Windows. See install.md for other platforms.")
    if sys.implementation.name != "cpython" or sys.version_info < (3, 10) or struct.calcsize("P") != 8:
        raise ValueError("Use standard 64-bit CPython 3.10 or newer; Python 3.12 is recommended. See install.md.")


def prepare_environment(directory, *, skip_install=False, root=ROOT):
    directory = Path(directory).resolve()
    executable = directory / "Scripts/python.exe"
    if not executable.is_file():
        if skip_install:
            raise ValueError("--skip-install requires an existing, installed environment. Run without it first.")
        if directory.exists():
            raise ValueError(f"{directory} already exists but has no working venv Python. "
                             "Set the copied/partial folder aside or choose another --venv-dir. Nothing was deleted.")
        print(f"[1/4] Creating isolated environment: {directory}", flush=True)
        venv.EnvBuilder(with_pip=True, system_site_packages=False).create(directory)
    else:
        print(f"[1/4] Checking existing environment: {directory}", flush=True)

    probe = (
        "import json,struct,sys; print(json.dumps({'prefix':sys.prefix,"
        "'base_prefix':sys.base_prefix,'version':list(sys.version_info[:2]),"
        "'bits':struct.calcsize('P')*8,'implementation':sys.implementation.name}))"
    )
    try:
        result = subprocess.run([str(executable), "-c", probe], cwd=root,
                                env=child_environment(), check=True, capture_output=True,
                                text=True, timeout=30)
        info = json.loads(result.stdout)
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        raise ValueError(f"Cannot use {directory}. It may be a copied or broken environment. "
                         "Set it aside and rerun, or use --venv-dir with a new folder. Nothing was deleted.") from exc
    if (Path(info["prefix"]).resolve() != directory
            or info["prefix"] == info["base_prefix"]
            or info["bits"] != 64 or info["implementation"] != "cpython"
            or tuple(info["version"]) < (3, 10)):
        raise ValueError("The selected directory is not a supported local 64-bit Python venv. "
                         "Create a fresh one using Python 3.12; no existing files were removed.")
    return executable


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, allow_abbrev=False,
        epilog="Other arguments are passed to Viser, e.g. --scene classroom --port 8081.")
    parser.add_argument("--venv-dir", type=Path, default=ROOT / ".venv", help="Environment folder; relative paths use the project root")
    parser.add_argument("--setup-only", action="store_true", help="Prepare/check dependencies without launching Viser")
    parser.add_argument("--skip-install", action="store_true", help="Reuse installed dependencies without contacting pip")
    parser.add_argument("--no-browser", action="store_true", help="Print the viewer URL without opening a browser tab")
    args, viewer_arguments = parser.parse_known_args(argv)
    try:
        check_bootstrap_python()
        for path in (*REQUIREMENTS, "requirements.txt", "scripts/run_viser.py", "mujoco/mearm.xml", "scenes/empty.json"):
            if not (ROOT / path).is_file():
                raise ValueError(f"Missing project file: {path}. Extract the complete current project bundle first.")
        directory = args.venv_dir if args.venv_dir.is_absolute() else ROOT / args.venv_dir
        executable = prepare_environment(directory, skip_install=args.skip_install)
        if args.skip_install:
            print("[2/4] Reusing installed packages (--skip-install).", flush=True)
        else:
            print("[2/4] Installing Python dependencies (internet required on first run).", flush=True)
            run([executable, "-m", "ensurepip", "--upgrade"])
            run([executable, "-m", "pip", "install", "--upgrade", "pip"])
            command = [executable, "-m", "pip", "install"]
            for filename in REQUIREMENTS:
                command.extend(["-r", str(ROOT / filename)])
            run(command)
        print("[3/4] Checking dependencies and loading the empty MuJoCo scene.", flush=True)
        if not args.skip_install:
            run([executable, "-m", "pip", "check"])
        run([executable, "-c", "import mujoco,viser,numpy,trimesh,serial; "
             "print('MuJoCo',mujoco.__version__,'| Viser',viser.__version__,'| NumPy',numpy.__version__)"])
        run([executable, str(ROOT / "scripts/run_mujoco.py"), "--headless", "--seconds", "0.05"])
        if args.setup_only:
            print(f"[4/4] Setup complete. Python: {executable}", flush=True)
            next_command = [str(ROOT / "setup_windows.cmd"), "--skip-install"]
            if directory.resolve() != (ROOT / ".venv").resolve():
                next_command.extend(["--venv-dir", str(directory.resolve())])
            print("To open the colored-cube viewer: " + subprocess.list2cmdline(next_command), flush=True)
            return 0
        print("[4/4] Starting Viser with colored cubes and pick-and-place controls.", flush=True)
        print("Keep this terminal open. Ctrl+C stops Viser. Open Pick and place in the browser panel.", flush=True)
        command = [executable, str(ROOT / "scripts/run_viser.py"), "--task", "cubes"]
        if not args.no_browser:
            command.append("--open-browser")
        command.extend(viewer_arguments)
        run(command)
        return 0
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 130
    except subprocess.CalledProcessError as exc:
        print(f"\nCommand failed (exit {exc.returncode}). Read the error above; see install.md for troubleshooting.", file=sys.stderr)
        return exc.returncode if exc.returncode > 0 else 1
    except (OSError, ValueError) as exc:
        print("\nSetup failed: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
