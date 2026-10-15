# Gazebo Sim

The supplied world targets **Gazebo Harmonic (gz-sim 8) or newer with the Bullet
Featherstone physics plugin**. It uses native SDFormat 1.11 mimic constraints for
the MeArm parallelogram compensation and mirrored jaws. Harmonic's default DART
engine does not implement these mimic constraints. The world explicitly selects
`gz-physics-bullet-featherstone-plugin`.

The model uses the same meshes, coordinates, masses and collision proxies as the
other formats in this project. This is a reduced mechanism: colocated compensation
joints reproduce the arm's coupled rotations instead of simulating every bolt and
closed linkage. All links, including the compensation and fixed servo links, have
positive mass and physical inertia. Servo cases and decorative parallel bars have
no collision proxies; the principal plates, arm bodies, tool and jaws do. Masses,
servo behavior and friction remain estimates. The `tool` frame is attached to
`hand` at the same TCP used in MuJoCo and URDF.

## Launch on Ubuntu / WSL

Install Gazebo using the [official Harmonic Ubuntu instructions](https://gazebosim.org/docs/harmonic/install_ubuntu/).
Ubuntu 22.04 or 24.04 is the supported baseline. From the project root, in a shell
where Gazebo is on PATH:

```sh
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r gazebo/world.sdf --physics-engine gz-physics-bullet-featherstone-plugin
```

The robot is fixed to the world; zero joint positions place its upper arm vertical,
forearm horizontal, tool level and jaws closed. The files reference local
`model://mearm_v1/meshes/...` assets and do not need an online model download.

Offline validation passed for all 14 links and meshes, 14 joints, 10 collision
proxies, five mimic constraints and four controllers. It checks positive definite,
physically valid inertias; model URI resolution; limits and mimic endpoints;
the tool frame; and four sampled poses against the analytical TCP position and
level-tool orientation. The model and world also passed a limited schema check
using 21 cached official SDFormat 1.11 description files. Run it again with:

```sh
python scripts/validate_gazebo.py --report docs/gazebo_validation.json
```

That check validates schema element names, attributes, scalar/vector types and
cardinalities for the constructs used here. It is a standard-library checker,
not libsdformat's parser. Plugin contents follow their official API documentation
and receive additional explicit configuration checks. The schema provenance and
SHA-256 hashes are in `references/sdformat-1.11/manifest.json`.

Gazebo is not installed in the environment where this package was generated, so
**`gz sdf -k` and Gazebo runtime behavior have not yet been verified**. Complete
these checks in a Gazebo installation:

```sh
gz sdf -k models/mearm_v1/model.sdf
gz sdf -k gazebo/world.sdf
gz sim -s -r --iterations 2000 gazebo/world.sdf \
  --physics-engine gz-physics-bullet-featherstone-plugin
```

If Gazebo reports the physics plugin missing, check your installed gz-physics
packages and the [official engine-selection guidance](https://gazebosim.org/api/sim/8/physics.html).
Changing to DART would remove the coupling behavior.

## Command the arm

In a second terminal with Gazebo on PATH, from the project root:

```sh
python scripts/gazebo_command.py --yaw 20 --shoulder 65 --forearm -20 --grip 15
python scripts/gazebo_command.py --demo
```

Inputs are geometric angles in **degrees**, not Arduino servo pulse values:

| Input | Meaning | Neutral |
| --- | --- | --- |
| `--yaw` | Rotation around the base's upward axis | 0° |
| `--shoulder` | Absolute upper-arm elevation above horizontal | 90° |
| `--forearm` | Absolute forearm elevation above horizontal | 0° |
| `--grip` | Opening rotation of each jaw | 0° |

The command helper reads the joint limits from `config/model_spec.json` and rejects
out-of-range requests before publishing. Use `--dry-run` to inspect the exact
commands without a Gazebo installation. The initial conservative ranges in this
model are not measured hardware stops.

Four independent joint controllers receive radians on:

```text
/model/mearm_v1/joint/yaw_joint/0/cmd_pos
/model/mearm_v1/joint/shoulder_joint/0/cmd_pos
/model/mearm_v1/joint/elbow_joint/0/cmd_pos
/model/mearm_v1/joint/left_finger_joint/0/cmd_pos
```

For example:

```sh
gz topic -t /model/mearm_v1/joint/yaw_joint/0/cmd_pos \
  -m gz.msgs.Double -p 'data: 0.35'
gz topic -e -t /world/mearm_demo/model/mearm_v1/joint_state
```

The controllers use `use_velocity_commands=true` with a speed cap of at most
0.8 rad/s. This gives ideal position tracking for STEM exercises; it does not
simulate calibrated SG90 torque saturation, backlash or servo electronics.
`initial_position` is the controller's initial target; the neutral posture is
encoded in the link poses. To return to neutral, run the helper without angle
arguments. Restart the world for a fresh simulation state.

## Windows

[Native Windows Gazebo instructions](https://gazebosim.org/docs/harmonic/install_windows/)
are experimental. Harmonic requires launching server and GUI in separate terminals.
In a PowerShell session with Gazebo installed:

```powershell
$env:GZ_SIM_RESOURCE_PATH = (Join-Path $PWD 'models')
gz sim -s -r gazebo/world.sdf --physics-engine gz-physics-bullet-featherstone-plugin
# A second terminal, with the same resource path:
gz sim -g
```

Use the MuJoCo model for the first local Windows run if a Gazebo environment is
not already available.

## Regenerate

After regenerating the shared model specification and meshes:

```sh
python scripts/build_gazebo.py
```

This writes `models/mearm_v1/model.sdf`, `models/mearm_v1/model.config` and
`gazebo/world.sdf`. The meshes remain shared with the other model formats.

The relevant upstream references are the [SDFormat mimic definition](https://sdformat.org/tutorials/pose_semantics_docs/mimic_proposal/1.11/),
[Harmonic mimic support](https://github.com/gazebosim/gz-harmonic/blob/main/highlights.md),
and [Gazebo joint position controller](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html).
