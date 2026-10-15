# Source provenance and licensing

Prepared 2026-10-15 for a MeArm Classic V1. Original notices inside source files
are retained. Generated meshes are derivatives; they are not claimed as wholly
original designs.

| Source | Pinned revision | Local files |
| --- | --- | --- |
| [MeArm official hardware](https://github.com/MeArm/MeArm) | `35398456ff3f1cded211198909bcd571d86102b3` | `references/MeArmV1.1.dxf`, assembly PDF, hardware README |
| [MeArm Arduino](https://github.com/MeArm/MeArm-Arduino) | `a07609820aea851524e73239e21dded3004eda2e` | `references/Geometry.md`, `config.h`, `fk.cpp`, `ik.cpp`, `MeArm.h`, `MeArm.cpp` |
| [Jakub Sikora's MeArm ROS model](https://github.com/kubasikora/mearm_model) | `280b8a10a000af25eeb7d7fe09f39df87920d8bc` | `references/community/` |

The official hardware README releases hardware under **Creative Commons
Attribution-ShareAlike 3.0**. Credit MeArm / Ben Gray / phenoptix and preserve the
share-alike terms for hardware derivatives such as the official DXF extrusions.
[CC BY-SA 3.0 license](https://creativecommons.org/licenses/by-sa/3.0/).
The README distinguishes code and photographic image terms; do not assume a
single license covers every resource. The included assembly guide retains its
original credits and notices.

The Arduino implementation files carry MIT notices with their original authors;
those notices remain in the included files. The geometry configuration is used
as dimensional evidence; new Python kinematics and generators are independently
implemented.

The community `package.xml` declares **MIT**, but its repository contains no
standalone LICENSE file. Its README credits meshes to
[Thingiverse design 616239](https://www.thingiverse.com/thing:616239).
That page could not be read during preparation, so the specific license for
those STL derivatives has **not been verified**. Preserve this attribution and
check the original design's terms before publicly redistributing the assembled
community-derived meshes. The package's MIT declaration is not presented as
proof of the upstream STL license.

Changes to the community assembly: bake mm-scale sheet coordinates and visual
origins into local meter-scale meshes; normalize frame rotations; use official
80 mm upper/forearm lengths; define an Arduino-compatible TCP; replace inertias,
limits and contact meshes; add four controllers, passive constraints and servo
body approximations. The source URDF and STL files remain unmodified in
`references/community/`.

New Python generators, tests and documentation in this project may be used under
the MIT license in `LICENSE`. That grant covers only the new code and prose;
it does not override the third-party hardware and mesh terms above.
