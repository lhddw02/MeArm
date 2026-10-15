# Geometry and coordinate conventions

All generated model dimensions are meters, masses kilograms, time seconds and
joint coordinates radians. The world is right-handed: **X forward, Y left,
Z up**. The base is fixed to the floor.

## Dimensions and evidence

| Parameter | Default | Evidence / limitation |
| --- | --- | --- |
| Shoulder to elbow | 80 mm | Official Arduino `config.h` |
| Elbow to wrist | 80 mm | Official Arduino `config.h`; community uses 81 mm + 1 mm offset, retargeted here |
| Combined base/hand offset | 68 mm | Official configuration for versions 0.3–1.2; current V3 default 22 mm is not used |
| Yaw to shoulder, horizontal | 14 mm | Supplied community URDF assembly, not an official kit measurement |
| Wrist to TCP | 54 mm | 68−14 mm; a tool convention within the jaws, different from the community tool site at 43 mm |
| Shoulder above floor | 55 mm | Community 24+29 mm, plus 2 mm to place its base bottom on the floor |
| Acrylic thickness | 3 mm | Official hardware repository |

The community plate and servo variants may differ from your kit. Assembled meshes
preserve that reference's shapes; upper and forearm lengths are retargeted to the
official 80 mm spans. The forearm's 1 mm transverse pivot offset is idealized away.
The separate `cad/` exports come from the official V1.1 DXF, not from the community
STLs. They are useful CAD starting points rather than a dimensionally verified
replacement for the physical kit.

The original DXF declares **centimeters**, so its coordinates are multiplied by
0.01 to obtain meters. Etch geometry is excluded from solids. The extraction
joins cut endpoints within 0.12 mm and bridges remaining gaps up to 2.2 mm to
close open cutting contours. Curves are tessellated; these repaired exports
also open a point-touching hole boundary by 0.002 mm where needed for a manifold
solid. They
should not be treated as manufacturing masters. Use the included original DXF
for manufacturing. Small uncontained faces below 50 mm² are omitted as scrap.
The contour IDs are generated IDs, not the printed assembly part numbers.

## Four independent coordinates

Let ψ be base yaw, α upper-arm elevation and β forearm elevation, both measured
**absolutely from horizontal**. Let γ be the opening rotation of the left jaw.

| Joint / actuator | Coordinate | Initial range |
| --- | --- | --- |
| `yaw_joint` / `base` | ψ | −45° … +45° |
| `shoulder_joint` / `shoulder` | α−90° | −45° … +45° (α 45° …135°) |
| `elbow_joint` / `elbow` | β | −45° … +45° |
| `left_finger_joint` / `gripper` | γ | 0° …28.65° |

Zero coordinates give α90°, β0°, a level hand, and closed jaws. Neutral TCP is
(148, 0, 135) mm. Position targets in the MuJoCo actuator panel use radians in
the coordinates above. The Gazebo helper accepts geometric degrees.

An elbow compensation joint rotates by the negative shoulder delta before the
forearm's independent β rotation. The wrist rotates by −β. This decomposition
allows each passive constraint to depend on one leader, matching Gazebo's native
mimic format. Two extra visible rods follow the corresponding leader joints;
their ideal coupling replaces a detailed closed-loop mechanical assembly.
The right jaw rotates by −γ.

MuJoCo imposes these relations with soft equality constraints; very small errors
are expected under load. Gazebo uses native mimic constraints with Bullet
Featherstone. The URDF uses the same mimic tree for ROS visualization; importing
URDF into a physics engine does not guarantee that engine honors the relations.

## Forward kinematics and Arduino mapping

With L1=L2=0.080, a=0.014, b=0.054 and h=0.055:

```text
r = a + L1 cos(α) + L2 cos(β) + b
x = r cos(ψ)
y = r sin(ψ)
z = h + L1 sin(α) + L2 sin(β)
```

The Arduino library uses +Y forward, +X right, and its origin at shoulder height,
according to `Geometry.md` and `fk.cpp`. Its README's ground-height wording is
inconsistent with those files. Convert in millimeters:

```text
x_sim = y_arduino / 1000
y_sim = -x_arduino / 1000
z_sim = z_arduino / 1000 + h
ψ = -a0; α = a1; β = a2
```

The library's default servo-degree calibration is below. These are calibration
values, not measured mechanical stops; clone servos and horns can differ.

```text
base servo     = 90 - 2 a0_deg = 90 + 2 ψ_deg
shoulder servo = 166 - (16/15) α_deg
elbow servo    = 90 + (6/5) β_deg
claw servo    = 180 - (7/6) a_claw_deg
```

The Arduino `a_claw` scale 0…90° is not this model's γ. The model does not yet
derive a calibrated servo-to-gear jaw travel relation; γ directly opens the
mirrored jaws. Hardware servo commands require calibration.

## Dynamics and contact

Plate inertias are computed from watertight local meshes with uniform acrylic
density 1180 kg/m³. Four approximate servo bodies use 9 g each. Positive full
inertia tensors are exported in all formats; no source placeholder 1 kg·m²
inertias are copied. Actual component masses should replace these estimates.

MuJoCo uses capped position actuators (0.15 N·m estimate) and damping; Gazebo's
demonstration uses ideal velocity-based position control capped at 0.8 rad/s.
Their servo dynamics are deliberately not equivalent. They share geometry,
kinematic constraints and approximate body inertias. Neither is a calibrated
electrical motor model.

Box contact proxies avoid taking a convex hull of a complete concave linkage
mesh. Self-collision is disabled. The fingers' contact proxies are simple pads,
not the exact toothed profile; grasp/contact behavior requires tuning and tests
with the intended object.

Sources: [official geometry](https://github.com/MeArm/MeArm-Arduino/blob/a07609820aea851524e73239e21dded3004eda2e/Geometry.md),
[official dimensions](https://github.com/MeArm/MeArm-Arduino/blob/a07609820aea851524e73239e21dded3004eda2e/config.h),
[forward kinematics](https://github.com/MeArm/MeArm-Arduino/blob/a07609820aea851524e73239e21dded3004eda2e/fk.cpp),
[servo calibration](https://github.com/MeArm/MeArm-Arduino/blob/a07609820aea851524e73239e21dded3004eda2e/MeArm.h),
[community assembly](https://github.com/kubasikora/mearm_model/blob/280b8a10a000af25eeb7d7fe09f39df87920d8bc/urdf/robot.urdf).
