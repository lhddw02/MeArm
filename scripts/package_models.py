"""Create a portable model bundle, excluding local runtimes and scratch files."""
from pathlib import Path
import hashlib, zipfile

ROOT=Path(__file__).resolve().parents[1]
FOLDERS=('arduino','cad','config','docs','examples','gazebo','models','mujoco','references','scripts','tests','urdf')
TOP=('README.md','LICENSE','THIRD_PARTY.md','requirements.txt','requirements-hardware.txt','.gitignore','start_mujoco.ps1')

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    inputs=[p for p in (ROOT/'references').rglob('*') if p.is_file() and p.name!='SHA256SUMS']
    (ROOT/'references/SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(ROOT).as_posix()}\n' for p in sorted(inputs)))
    paths=[ROOT/p for p in TOP]
    for folder in FOLDERS:
        paths.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
    paths=sorted(set(paths))
    manifest=ROOT/'MANIFEST.sha256'
    manifest.write_text(''.join(f'{digest(p)}  {p.relative_to(ROOT).as_posix()}\n' for p in paths))
    paths.append(manifest)
    target=ROOT/'dist/mearm_v1_simulation.zip'; target.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for p in paths: archive.write(p,Path('mearm_v1_simulation')/p.relative_to(ROOT))
    print(f'{target}: {len(paths)} files, {target.stat().st_size/1048576:.2f} MiB')

if __name__=='__main__': main()
