# Cached official SDFormat description schemas

These 21 small files are the subset of the native SDFormat 1.11 description
schemas used by this project's generated model and world. They were downloaded
from the official [gazebosim/sdformat repository, sdf14 branch](https://github.com/gazebosim/sdformat/tree/sdf14/sdf/1.11)
on 2026-10-15. The branch is mutable; `manifest.json` records the exact bytes via
SHA-256 hashes. These are upstream files, not schemas authored for this project.

Upstream SDFormat is distributed under the Apache License 2.0; see `LICENSE`.

`scripts/validate_gazebo.py` reads the cache without network access. Its schema
check is intentionally limited to element names, attributes, value types and
cardinalities encountered in this model. Gazebo plugin parameters are extensible
data in the schema and are checked separately. This does not substitute for
libsdformat's full semantic frame validation, `gz sdf -k`, or a running Gazebo
physics test.

To retrieve a missing file and regenerate the hash manifest explicitly:

```sh
python scripts/validate_gazebo.py --fetch-schema
```
