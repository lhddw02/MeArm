"""Keep optional development wheels out of incompatible Python environments."""
from pathlib import Path
import os
import sys
import sysconfig


def _compatible_tag(tag):
    """Conservatively accept only matching native wheels or CPython stable ABI."""
    interpreter, abi, platform = tag.split('-')
    current = f'cp{sys.version_info.major}{sys.version_info.minor}'
    current_platform = sysconfig.get_platform().replace('-', '_').replace('.', '_')
    if platform != current_platform:
        return False
    if interpreter == current and abi == current:
        return True
    if abi == 'abi3' and interpreter.startswith('cp3'):
        return sys.version_info.major == 3 and int(interpreter[3:]) <= sys.version_info.minor
    return False


def configure_dependencies(root):
    """Prefer installed packages; use compatible .deps only outside Conda/venv."""
    if (os.environ.get('CONDA_PREFIX') or (Path(sys.prefix)/'conda-meta').is_dir()
            or sys.prefix != sys.base_prefix):
        return False
    deps = Path(root)/'.deps'
    wheels = list(deps.glob('*.dist-info/WHEEL'))
    if not wheels or sys.implementation.name != 'cpython':
        return False
    for wheel in wheels:
        lines = wheel.read_text(encoding='utf-8').splitlines()
        if 'Root-Is-Purelib: true' in lines:
            continue
        tags = [line[5:].strip() for line in lines if line.startswith('Tag: ')]
        if not tags or not any(_compatible_tag(tag) for tag in tags):
            return False
    # Append so the interpreter's installed packages always take precedence.
    if str(deps) not in sys.path:
        sys.path.append(str(deps))
    return True
