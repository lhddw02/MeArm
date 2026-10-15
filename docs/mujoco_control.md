# Moving the MeArm in MuJoCo

Use your activated `mearm` environment and run these commands from the project
directory. There are four independent controls: base yaw, shoulder elevation,
forearm elevation and gripper opening. The other joints follow the linkage
constraints automatically.

## Mouse controls in MuJoCo

```bat
python scripts/run_mujoco.py
```

Expand **Control** in the right panel and move the `base`, `shoulder`, `elbow`
and `gripper` sliders. **Shift+Tab** toggles that panel; **Alt+C** toggles the
Control section. The sliders command position actuators and the physics moves
the arm towards those positions. Use Control for live motion rather than
editing the passive linkage coordinates in the Joint section.

Do not use `--demo` for manual operation: the demo writes a new target every
simulation step, overwriting manual slider changes.

The native slider values are **radians**, with these model coordinates:

| Slider | Meaning | Range | Example |
| --- | --- | --- | --- |
| `base` | Yaw about the upward axis | −0.7854…0.7854 | 0.3491 = left20° |
| `shoulder` | Upper-arm elevation minus90° | −0.7854…0.7854 | −0.4363 = elevation65° |
| `elbow` | Absolute forearm elevation | −0.7854…0.7854 | −0.3491 = elevation−20° |
| `gripper` | Opening angle of each mirrored jaw | 0…0.5 | 0.2618 = open15° |

See the [official MuJoCo viewer reference](https://mujoco.readthedocs.io/en/stable/programming/samples.html)
for the viewer controls and shortcuts.

## Live commands in degrees

```bat
python scripts/run_mujoco.py --interactive
```

Keep the MuJoCo window open and type in the **terminal** where you started it:

```text
yaw 20
shoulder 65
forearm -20
grip 15
```

Each command changes only that servo target and preserves the other three,
including targets you changed with the native sliders. Set all four together:

```text
joints 20 65 -20 15
```

Order: **yaw, shoulder, forearm, grip**, all in degrees. Geometric neutral is
`joints 0 90 0 0`. `home` returns there; `status` prints the commanded angles,
the tool target from forward kinematics and the measured tool position.
`help` lists commands; `quit` closes the simulation.

The command ranges are yaw−45…45°, shoulder45…135°, forearm−45…45° and
grip0…28.6479°. Shoulder and forearm are both elevations measured from
horizontal; the forearm command is not the bend angle between the two links.

## Reach an XYZ position

In interactive mode:

```text
xyz 120 30 120
status
xyz 120 -30 120 15
```

Coordinates are **millimeters in the world frame**:

- X: forward.
- Y: left.
- Z: height above the floor.

The optional fourth value sets the jaw opening in degrees. Without it, the
current gripper command is preserved. The neutral tool position is
(148,0,135)mm. These are the project simulation coordinates; the Arduino
library uses a different convention, described in [kinematics.md](kinematics.md).

The controller solves inverse kinematics and verifies the actuator limits
before changing any command. A target outside the allowed workspace is
rejected without partially moving the arm. Targets are applied through the
position actuators; the arm moves and settles under physics rather than being
teleported to the solution. `status` distinguishes the target position from
the measured position, so allow time to settle before comparing them.

The gripper remains level. This model has three positioning degrees of freedom
plus the gripper, so it cannot independently command tool orientation. XYZ
commands solve the endpoint; they do not plan collision-free paths.

## Set a pose when launching

```bat
python scripts/run_mujoco.py --joints 20 65 -20 15
python scripts/run_mujoco.py --target-mm 120 30 120
python scripts/run_mujoco.py --interactive --target-mm 120 30 120
```

For a run without a window:

```bat
python scripts/run_mujoco.py --headless --target-mm 120 30 120 --seconds 3
```

## Write your own movement program

Run and edit `examples/control_arm.py`. Its main operations are:

```python
arm = MeArmController(model, data)
arm.set_joint_targets(yaw_deg=20, shoulder_deg=65, forearm_deg=-20, grip_deg=15)
arm.move_to(120, 30, 120)  # world millimeters; preserve gripper opening
```

Continue calling `mujoco.mj_step(model,data)` to execute the commands. The helper
resolves actuator names rather than relying on numeric indices, validates the
whole request and writes only `data.ctrl`. It leaves passive joints to their
constraints. A partial call such as `arm.set_joint_targets(yaw_deg=-20)`
preserves the remaining actuator commands.
