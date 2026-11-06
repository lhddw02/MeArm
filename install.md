# Install MeArm simulation on a new Windows computer

Start with **Viser**: it runs the MeArm simulation in a browser, using MuJoCo
for physics. This guide also covers the native MuJoCo viewer and optional
Gazebo setup. Every viewer defaults to an empty background with the arm and a
support floor; colored cubes and room backgrounds are selected explicitly.

The project does **not** include a portable Python environment. Create `.venv`
on the new computer and install its packages from the requirements files.
Python environments contain machine-specific interpreter paths and compiled
packages; copying another computer's `.venv` or `.deps` is not a replacement
for installation. See the [Python virtual-environment documentation](https://docs.python.org/3.12/library/venv.html).

## Automatic setup and launch

After installing 64-bit Python as described in section 2 and extracting the
complete project, double-click **`setup_windows.cmd`** in the project folder.
You can also run it from a normal PowerShell terminal:

```powershell
.\setup_windows.cmd
```

The launcher performs these steps:

1. Finds Python, preferring Python 3.12 through the Windows launcher, then a
   working project `.venv` or `python` on PATH.
2. Creates an isolated project `.venv` if absent, or checks/reuses an existing
   valid local environment. An unusable copied or broken environment produces
   an actionable error; the launcher does not delete it.
3. Installs `requirements-viser.txt` and `requirements-hardware.txt`, checks the
   Python dependencies, and loads the MuJoCo model for a brief physics check.
4. Launches Viser with `--task cubes` and opens its actual URL in your default
   browser. The background remains empty unless you select another scene.

Internet is needed for the initial package downloads. The terminal stays open
while Viser runs. In the browser, expand **Pick and place**, choose a cube color,
set the destination center (for example `180, -45, 90` mm), and click **Pick and
place selected cube**. No robot hardware is connected or enabled by this script.
Gazebo, WSL and Arduino IDE are separate optional installations, not Python
dependencies installed by this launcher.

Useful options:

```powershell
# Choose a background; other Viser arguments are forwarded too.
.\setup_windows.cmd --scene classroom

# Faster later launch: reuse installed packages without running pip.
.\setup_windows.cmd --skip-install

# Install/check only, without launching the server or browser.
.\setup_windows.cmd --setup-only

# Launch without opening a browser tab automatically.
.\setup_windows.cmd --skip-install --no-browser --port 8081

# Choose another local environment folder instead of .venv.
.\setup_windows.cmd --venv-dir ".venv-viser"
```

Options such as `--cube red --place-mm 180 -45 90` can also be forwarded to start
a simulated job immediately. For manual control on launch, omit those job flags.
Stop an old server with **Ctrl+C** before starting its replacement. If no browser
tab opens, use the printed URL. An error window waits for a key so its message
can be read; automated runs may set `MEARM_NO_PAUSE=1` to disable that pause.

If Python discovery selects the wrong installation, specify a full executable
path for this terminal session:

```powershell
$env:MEARM_PYTHON = "C:\Path To\Python312\python.exe"
.\setup_windows.cmd
```

Standard 64-bit CPython 3.10+ is accepted; 3.12 is recommended for a new setup.
The `.cmd` entry point does not need `Activate.ps1` or a PowerShell execution-policy
change. Its standard-library helper is `scripts/setup_windows.py`. The manual
instructions below remain available if you prefer to perform each step yourself.

## 1. What you need

This walkthrough targets **64-bit Windows on an Intel/AMD computer**, with
Python **3.12, 64-bit**, and a current Edge, Chrome, or Firefox browser with
WebGL support. Windows 11 is the baseline for the optional WSL GUI workflow.
Use your graphics vendor's current Windows driver if 3D rendering is unavailable.

You need internet access for the initial Python/package downloads and the full
project folder, including `scripts`, `models`, `mujoco`, `config`, and `scenes`.
The prebuilt models are included; you do not need to regenerate CAD files.
Arduino hardware, Conda, Docker, ROS, and Gazebo are not needed to run Viser.

| What you want to run | Setup |
| --- | --- |
| Viser browser simulation, including pick-and-place | Sections 2–6; install `requirements-viser.txt` |
| Native MuJoCo viewer | Same environment; section 7 |
| Gazebo Sim | Separate Ubuntu/WSL installation; section 8 |
| Arduino command synchronization | Optional additional package and firmware; section 9 |

## 2. Install Python

Install the **Python install manager** from the
[official Python Windows downloads page](https://www.python.org/downloads/windows/).
Open a **new, normal PowerShell terminal** after installation, then install
the standard Python 3.12 runtime:

```powershell
py install 3.12
py -3.12 --version
py -3.12 -c "import struct; print('Python bits:', struct.calcsize('P') * 8)"
```

The last command should print `Python bits: 64`. These instructions select 3.12
explicitly so a different default Python version does not change this setup.
Use the standard runtime, not an embeddable or free-threaded build.
The install-manager commands are described in the
[official Windows Python guide](https://docs.python.org/3/using/windows.html).

If you already installed standard 64-bit Python 3.12 using its conventional
installer, `py -3.12 --version` may work immediately; skip `py install 3.12`.
An older `py` launcher does not implement the `install` subcommand. If the new
manager is also installed, `pymanager install 3.12` is its unambiguous equivalent.
If your installation provides only `python`, confirm `python --version` is 3.12
and use `python -m venv .venv` in the next section.

## 3. Get the project and create a fresh environment

Copy/extract the complete project ZIP, or download the repository using
**Code → Download ZIP** from [the MeArm repository](https://github.com/lhddw02/MeArm).
Git is optional. If using the packaged `mearm_v1_simulation.zip`, extract it first;
do not run scripts from inside the compressed-folder view.
Use a revision containing `requirements-viser.txt` and `scripts/run_viser.py`.
If a repository download is older and lacks either file, obtain the updated
complete bundle; installing Python alone will not add the missing project files.

Choose a normal writable folder, for example
`C:\Users\YOUR_NAME\Documents\MeArm`. Enter the folder that directly contains
`README.md` and `requirements-viser.txt`; ZIP extraction can create an extra
nested directory. Substitute your actual location in the first command:

```powershell
cd "$env:USERPROFILE\Documents\MeArm"
Get-ChildItem README.md, requirements-viser.txt, scripts, mujoco, models, scenes
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements-viser.txt
.\.venv\Scripts\python.exe -m pip check
```

`pip check` should report `No broken requirements found`. The requirements
install Viser, MuJoCo, NumPy, mesh/image support, and the project's model tools.
There is no separate MuJoCo desktop installer for this workflow.

The default `venv` command creates an isolated environment. Do not add
`--system-site-packages`: the new installation should not depend on packages
from a different Python or Conda environment. If a copied `.venv` is already
present, set it aside before creating the new one.

**Activation is optional.** All Windows commands here call the environment's
Python executable directly. You do not need to run `Activate.ps1`, change
PowerShell execution policy, or use `start_mujoco.ps1`. Run these commands in
PowerShell, not Bash; the Ubuntu commands later in this guide are separate.

Check that imports come from the new environment:

```powershell
.\.venv\Scripts\python.exe -c "import sys, mujoco, viser, numpy; print(sys.executable); print('MuJoCo:', mujoco.__version__); print('Viser:', viser.__version__); print('NumPy:', numpy.__version__)"
.\.venv\Scripts\python.exe scripts/run_mujoco.py --headless --demo --seconds 2
```

The executable path should end in `MeArm\.venv\Scripts\python.exe`. The second
command checks model loading and physics without opening a graphics window.

## 4. Run Viser

From the project root:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py
```

Keep that terminal running. Open the printed URL in your browser, normally
**http://127.0.0.1:8080**. If another process is using that port, Viser may select
the next available one; use the URL actually printed. To choose a port explicitly:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py --port 8081
```

The initial scene contains the arm and a plain floor. Use the **Joint controls**
sliders, or enter XYZ in **Tool target** and click **Reach target**. GUI joint
angles are degrees; tool coordinates are millimetres, with X forward, Y left,
and Z up. **Home**, **Joint demo**, **Pause physics**, and **Reset physics** are
available in the panel. Stop the server with **Ctrl+C** in its terminal.

Start it again after changing Python code or updating the project. Refresh an
old browser tab with **Ctrl+F5** to avoid viewing a stale session. Each time you
open a new terminal, `cd` back to the project root; package installation is not
needed again unless requirements change or you recreate the environment.

## 5. Colored cubes and automatic pick-and-place

Stop the arm-only server, then launch the cube task:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py --task cubes
```

Open **Pick and place**, choose **red, green, blue, or yellow**, and set the
destination cube center. A useful first destination is **X=180, Y=-45, Z=90 mm**.
Click **Pick and place selected cube**. The four cubes are 20 × 20 × 20 mm and
sit on an 80 mm-high platform, so their supported center height is **Z=90 mm**.

The panel displays the phase and measured cube centers; a green square marks
the destination. Manual commands and scene switching are locked while the task
runs. **Pause physics** freezes the sequence, **Cancel pick and place** restores
manual control, and **Reset physics** restores the original cube positions.
Choose an unoccupied tabletop destination inside the arm's supported workspace.

To start a job directly from the terminal:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py --task cubes --cube red --place-mm 180 -45 90
```

Replace `red` with another color. Run one job at a time. The simulation uses
MuJoCo contact and friction to carry the cube; status reports a failed grasp or
placement if the physical sequence does not complete. Full details are in the
[pick-and-place guide](docs/pick_and_place.md) and
[Viser scene guide](docs/scene_environments.md).

## 6. Select or import a scene

An empty background is the default. Select a preset or your own file with
`--scene`; the same values work in the native MuJoCo viewer:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py --scene classroom
.\.venv\Scripts\python.exe scripts/run_viser.py --task cubes --scene workshop
.\.venv\Scripts\python.exe scripts/run_viser.py --scene scenes/imported_room.json
.\.venv\Scripts\python.exe scripts/run_viser.py --scene "C:/MyScenes/room.glb"
.\.venv\Scripts\python.exe scripts/run_viser.py --scene "C:/MyScenes/lab.jpg"
```

These are separate launch examples; stop a server before starting its replacement.
Use the Viser **Environment** controls to select a preset or load a server-local
path during a session. Supported mesh formats are GLB, GLTF, OBJ, STL and PLY;
PNG/JPG files become flat world-space backdrops. JSON layouts combine assets and
primitives. Keep companion texture/buffer files with OBJ/GLTF assets.

Background coordinates use metres and Z up. For a millimetre-scale mesh, for example:

```powershell
.\.venv\Scripts\python.exe scripts/run_viser.py --scene "C:/MyScenes/room.obj" --scene-scale 0.001
```

`--scene-pos X Y Z` adjusts placement in metres and `--scene-rpy R P Y` adjusts
orientation in degrees. Imports are visual-only by default. See
[scene formats, placement and collision proxies](docs/scene_environments.md)
before adding physical obstacles.

## 7. Native MuJoCo viewer

The Viser environment already includes everything needed for these commands.
For a MuJoCo-only installation, create `.venv` as above and install
`requirements.txt` instead of `requirements-viser.txt`.

```powershell
.\.venv\Scripts\python.exe scripts/run_mujoco.py
.\.venv\Scripts\python.exe scripts/run_mujoco.py --interactive
.\.venv\Scripts\python.exe scripts/run_mujoco.py --demo --scene classroom
```

Run one example at a time. In the native viewer, expand **Control** in the right
panel; Shift+Tab toggles that panel. Native sliders use radians and the shoulder
is an offset from vertical. With `--interactive`, type terminal commands such as
`joints 20 65 -20 15` or `xyz 120 30 120`. The `--demo` mode owns the joint targets;
omit it for manual control.

For the native colored-cube task:

```powershell
.\.venv\Scripts\python.exe scripts/run_cube_scene.py --mode manual
.\.venv\Scripts\python.exe scripts/run_cube_scene.py --mode auto --cube blue --place-mm 180 -45 90
```

For an offscreen still or a finite physics check:

```powershell
.\.venv\Scripts\python.exe scripts/run_mujoco.py --scene workshop --render results/workshop.png
.\.venv\Scripts\python.exe scripts/run_mujoco.py --headless --demo --seconds 5
```

The native viewer and `--render` need a working OpenGL graphics driver. The
headless physics check does not create a rendering window. More controls are in
[MuJoCo control](docs/mujoco_control.md) and [cube-scene setup](docs/cube_scene.md).

## 8. Optional Gazebo Sim through WSL2

Gazebo has a separate Linux installation; Windows `.venv` packages do not install
Gazebo. The supplied world targets **Gazebo Harmonic with Bullet Featherstone**.
Gazebo runtime behavior has not been validated on the development machine;
the project's existing Gazebo checks cover model/schema structure.

### Windows step: install Ubuntu

In **PowerShell as Administrator**, use the
[Microsoft WSL installation procedure](https://learn.microsoft.com/en-us/windows/wsl/install):

```powershell
wsl --install -d Ubuntu-24.04
```

Restart Windows if requested, then open **Ubuntu 24.04** and complete its Linux
user setup. Use WSL2 with WSLg for Gazebo's GUI. Microsoft documents its Windows
and GPU requirements in [Run Linux GUI apps with WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

### Ubuntu step: install Gazebo

The following commands run in the **Ubuntu terminal**, not PowerShell. They
follow the [official Harmonic Ubuntu installation](https://gazebosim.org/docs/harmonic/install_ubuntu/):

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg python3
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

Enter the project using its Linux path. Replace `YOUR_WINDOWS_USER` with your
Windows account's folder name, or use the actual path where you extracted it:

```bash
cd "/mnt/c/Users/YOUR_WINDOWS_USER/Documents/MeArm"
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sdf -k models/mearm_v1/model.sdf
gz sdf -k gazebo/world.sdf
gz sim -r gazebo/world.sdf --physics-engine gz-physics-bullet-featherstone-plugin
```

In a second **Ubuntu** terminal, enter the same project directory and command it:

```bash
cd "/mnt/c/Users/YOUR_WINDOWS_USER/Documents/MeArm"
python3 scripts/gazebo_command.py --yaw 20 --shoulder 65 --forearm -20 --grip 15
```

This helper uses Python's standard library and Gazebo's command-line tools;
the Windows `.venv` is not used inside WSL. The Gazebo world starts with the arm,
ground and lighting. The shared JSON/image/mesh background loader and Viser
pick-and-place panel are features of the MuJoCo/Viser runners, not this Gazebo world.

If Featherstone is missing, inspect available plugin packages with
`apt-cache search 'gz-physics.*bullet'` and install the matching Bullet
Featherstone plugin for your Gazebo release. Follow the
[Gazebo engine-selection guide](https://gazebosim.org/api/sim/8/physics.html).
The arm's native mimic constraints require this engine; Harmonic's default DART
engine does not reproduce those couplings. For WSL graphics problems, verify
WSLg and your graphics driver first. A server-only check is:

```bash
gz sim -s -r --iterations 2000 gazebo/world.sdf --physics-engine gz-physics-bullet-featherstone-plugin
```

See [the full Gazebo guide](docs/gazebo.md) for model topics, schema checks and
the experimental native-Windows alternative. ROS is not required for this world.
The supplied `urdf/mearm.urdf` can also be integrated into an existing ROS/RViz
workflow; it is not a separate configured Windows physics simulator.

## 9. Optional Arduino synchronization

Pure simulation needs no Arduino connection. If you later connect the real arm,
install the serial dependency into the same Windows environment:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-hardware.txt
.\.venv\Scripts\python.exe scripts/sync_arduino.py --dry-run
```

Follow [hardware synchronization](docs/hardware_sync.md) for Arduino IDE/Servo
installation, wiring, measured calibration, firmware upload, port selection and
explicit arming. The Viser pick-and-place runner does not send servo commands to
real hardware. Standard hobby-servo telemetry reports commands, not measured
shaft angles.

## 10. Checks, updates and troubleshooting

Run these from PowerShell in the project root:

```powershell
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py"
.\.venv\Scripts\python.exe tests/validate_viser.py
```

The last command tests real local HTTP/WebSocket controls and all four
pick-and-place colors. It starts and stops a temporary server; it does not open
hardware ports or verify browser raster output. A visible Viser run remains the
graphics check on your new computer.

After obtaining updated project files, stop any running servers, then refresh
dependencies if needed:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-viser.txt
```

| Symptom | What to check |
| --- | --- |
| `py` is not recognized | Install Python and open a new PowerShell terminal. Check the install manager's troubleshooting guide. |
| `py install` is treated as a filename | You have the legacy launcher. Use an already installed 3.12 runtime or the manager's `pymanager install 3.12`. |
| `.venv\Scripts\python.exe` is missing | Enter the project root and create `.venv` in section 3. Extracting the project does not create it. |
| The venv refers to another user's Python path | It was copied from another computer. Set it aside and recreate it locally. |
| Scripts are disabled / `param([switch]...)` syntax error | Use the direct Python commands here; `.ps1` files are PowerShell scripts, not Bash scripts. |
| `No module named viser` or `mujoco` | Install requirements using the exact `.venv` Python executable used to launch the script. |
| NumPy `.pyd` / C-extension import error | Check Python version/64-bit architecture and package locations. Recreate a copied or mixed-version environment; do not copy `.deps` binaries into it. |
| `Could not open ...xml` or missing STL/scene files | Keep the complete project directory structure and run from its root. |
| Browser cannot connect | Keep the server terminal running and open its printed URL/port. A previous server may still own 8080. |
| Black Viser surfaces or striped shadows | Use the current lighting-fixed script, restart its server, then Ctrl+F5. Also check browser WebGL and your graphics driver. |
| Pick is rejected or fails | Read the phase/error; keep the destination on the tabletop, unoccupied, reachable, and at Z=90 mm. Reset physics for the original cube layout. |
| Package downloads fail | Check internet access and your network's proxy/package policy; confirm the terminal is using the intended interpreter. |
| Gazebo GUI does not appear | Run Ubuntu commands in WSL2 and check WSLg/GPU support; see the Gazebo guide. |

To inspect the interpreter and NumPy installation without changing anything:

```powershell
.\.venv\Scripts\python.exe -c "import sys, numpy; print(sys.executable); print(numpy.__file__)"
```

Keep requirements files with the source. To record the exact versions from a
working machine for a matching Python/Windows architecture, you may export them:

```powershell
.\.venv\Scripts\python.exe -m pip freeze > requirements-local-freeze.txt
```

Create a fresh `.venv` on another computer before installing that freeze file;
the environment directory itself remains machine-specific.

## Validation performed

The environment-recreation check on Windows x64 used a new isolated Python
3.12.14 environment with `include-system-site-packages = false`. Installing only
`requirements-viser.txt` resolved MuJoCo 3.17.0, Viser 1.1.1, NumPy 2.5.4 and
Trimesh 5.2.0. All 71 tests, the headless physics check, and the Viser WebSocket
controls/pick-and-place checks passed without the original `.venv`, `.deps` or
Conda packages. See [installation validation](docs/windows_install_validation.json).
This verifies environment recreation on the existing Windows machine; browser/GPU
behavior on your new device and the optional Gazebo runtime still need local checks.

## Daily launch commands

From your project directory, run one of these in PowerShell:

```powershell
# Viser: arm only, empty background
.\.venv\Scripts\python.exe scripts/run_viser.py

# Viser: colored cubes with GUI pick-and-place controls
.\.venv\Scripts\python.exe scripts/run_viser.py --task cubes

# Native MuJoCo: arm only, empty background
.\.venv\Scripts\python.exe scripts/run_mujoco.py
```
