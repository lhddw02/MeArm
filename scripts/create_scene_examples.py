"""Generate the small original GLB room used by the scene-import example."""
from pathlib import Path
import json

if __package__:
    from ._runtime import configure_dependencies
else:
    from _runtime import configure_dependencies
ROOT = Path(__file__).resolve().parents[1]
configure_dependencies(ROOT)
import trimesh


def main():
    source = json.loads((ROOT / "scenes/workshop.json").read_text())
    scene = trimesh.Scene()
    for item in source["objects"]:
        mesh = trimesh.creation.box(extents=item["size_m"])
        mesh.visual.vertex_colors = [round(v * 255) for v in item["rgba"]]
        pose = trimesh.transformations.translation_matrix(item["pos_m"])
        scene.add_geometry(mesh, node_name=item["name"], transform=pose)
    target = ROOT / "scenes/assets/demo_room.glb"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(scene.export(file_type="glb"))
    print(target)


if __name__ == "__main__":
    main()
