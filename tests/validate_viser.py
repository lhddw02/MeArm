r"""Exercise the real local HTTP/WebSocket server without browser automation.

Run explicitly: .venv\Scripts\python.exe tests/validate_viser.py
Requires local socket access. Does not open hardware ports or control a browser.
"""
from pathlib import Path
import json
import math
import sys
import threading
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import msgspec
import numpy as np
from PIL import Image
import viser
from websockets.sync.client import connect

from scripts.run_viser import ViserSimulation
from scripts.cube_scene import CUBES


def main():
    server = viser.ViserServer(host="127.0.0.1", port=0, verbose=False)
    checks = []
    try:
        app = ViserSimulation(server)
        assert all(light.intensity > 0 for light in app.lights)
        assert all(not light.cast_shadow for light in app.lights[1:])
        checks.append("Explicit ambient/key/fill lighting with shadows disabled")
        address = f"127.0.0.1:{server.get_port()}"
        with urlopen("http://" + address, timeout=3) as response:
            assert response.status == 200 and b"<html" in response.read().lower()
        checks.append("HTTP client page served")
        with connect("ws://" + address, subprotocols=["viser-v" + viser.__version__],
                     open_timeout=3, max_size=50*1024*1024) as client:
            assert len(client.recv(timeout=3)) > 0
            stop_receiving = threading.Event()
            def receive_messages():
                while not stop_receiving.is_set():
                    try:
                        client.recv(timeout=.1)
                    except TimeoutError:
                        continue
                    except Exception:
                        break
            receiver = threading.Thread(target=receive_messages, daemon=True)
            receiver.start()
            client.send(msgspec.msgpack.encode({
                "type": "ViewerCameraMessage", "wxyz": [1., 0., 0., 0.],
                "position": [-.42, -.4, .3], "fov": math.pi/4,
                "near": .001, "far": 100., "image_height": 600, "image_width": 800,
                "look_at": [.07, 0., .10], "up_direction": [0., 0., 1.]}))
            deadline = time.monotonic() + 3
            while not server.get_clients() and time.monotonic() < deadline:
                time.sleep(.01)
            assert server.get_clients()

            def update(handle, value=None, *, button=False):
                client.send(msgspec.msgpack.encode({"type": "GuiUpdateMessage", "uuid": handle._impl.uuid,
                                                    "updates": {} if button else {"value": value}}))

            def wait_for(predicate):
                deadline = time.monotonic() + 3
                while not predicate() and time.monotonic() < deadline:
                    time.sleep(.01)
                assert predicate(), "Timed out waiting for GUI event"

            update(app.sliders[0], 10.)
            wait_for(lambda: not app.commands.empty())
            app.advance(.3)
            assert abs(app.arm.target_degrees[0] - 10.) < 1e-9
            checks.append("WebSocket joint slider changed MuJoCo target")
            app.demo = True
            app.commands.put(("joint", (0, 5.)))
            app.process_commands()
            assert not app.demo and not app.demo_checkbox.value
            checks.append("Manual joint control disables demo in physics and GUI")
            for field, value in zip(app.xyz, (145., 0., 135.)):
                update(field, value)
            wait_for(lambda: app.xyz[0].value == 145.)
            update(app.reach_button, button=True)
            wait_for(lambda: not app.commands.empty())
            app.advance(.5)
            assert np.linalg.norm(app.arm.forward_kinematics() - [145., 0., 135.]) < 1e-6
            checks.append("WebSocket XYZ button solved IK and advanced physics")
            for reference in ("workshop", "classroom", "scenes/imported_room.json"):
                update(app.scene_path, reference)
                wait_for(lambda: app.scene_path.value == reference)
                update(app.load_button, button=True)
                wait_for(lambda: not app.commands.empty())
                app.advance(.1)
                assert app.scene_reference == reference, app.status.value
                assert "Ready" in app.status.value
                checks.append("Scene switch: " + reference)
            assert app.preset_dropdown.value == "Custom file"
            update(app.preset_dropdown, "empty")
            wait_for(lambda: not app.commands.empty())
            app.advance(.1)
            assert app.scene_reference == "empty" and not app.environment.visual_meshes
            checks.append("Preset selection returns custom import to empty")
            previous = app.renderer
            app.commands.put(("scene", "tmp/definitely_missing_scene.glb"))
            app.process_commands()
            assert app.renderer is previous and "Rejected" in app.status.value
            checks.append("Failed scene load preserved active model and nodes")
            background = ROOT / "tmp/viser_validation_background.png"
            Image.fromarray(np.tile(np.array([[[220, 80, 40], [40, 100, 220]]], dtype=np.uint8), (64, 64, 1))).save(background)
            app.commands.put(("scene", str(background)))
            app.advance(.1)
            assert len(app.renderer.static_nodes) == 1 and "Ready" in app.status.value
            checks.append("Textured image background added to real Viser server")
            assert not any(warning.number for warning in app.data.warning)
            checks.append("No MuJoCo warnings")
            app.renderer.remove()
            server.gui.reset()
            app = ViserSimulation(server, task="cubes")
            color_results = {}
            for cube in CUBES:
                color = cube['name']
                if color_results:
                    update(app.reset_button, button=True)
                    wait_for(lambda: not app.commands.empty())
                    app.advance(0)
                    assert app.pick_task.status == 'idle'
                update(app.cube_color, color)
                wait_for(lambda: app.cube_color.value == color)
                update(app.pick_button, button=True)
                wait_for(lambda: not app.commands.empty())
                app.advance(0)
                assert app.pick_task.active and app.pick_task.color == color, app.status.value
                assert app.sliders[0].disabled
                max_height = 0.
                for _ in range(260):
                    app.advance(.1)
                    max_height = max(max_height, app.pick_task.cube_position_mm[2])
                    if not app.pick_task.active:
                        break
                result = app.pick_task.report()
                assert result['status'] == 'complete', result
                assert result['released'] and result['placement_error_mm'] < 3.
                assert max_height > 105.
                assert not app.sliders[0].disabled
                geom_id = app.model.geom('cube_' + color + '_geom').id
                node = next(node for gid, node in app.renderer.nodes if gid == geom_id)
                assert np.linalg.norm(1000 * np.array(node.position) - [180, -45, 90]) < 3.
                assert not any(w.number for w in app.data.warning)
                result['peak_cube_height_mm'] = max_height
                result['rendered_cube_center_mm'] = (1000 * np.array(node.position)).tolist()
                color_results[color] = result
                checks.append('WebSocket pick/place completed and released: ' + color)
            pick_report = {"date": "2026-11-06", "viser_version": viser.__version__,
                           "method": "real Viser GUI events over WebSocket, MuJoCo pad contacts and friction, no pose edits or attachments",
                           "passed": True, "results": color_results,
                           "browser_raster_verified": False}
            (ROOT / 'docs/viser_pick_place_validation.json').write_text(json.dumps(pick_report, indent=2)+'\n', encoding='utf-8')
            stop_receiving.set()
            receiver.join(timeout=1)
        report = {"date": "2026-11-06", "viser_version": viser.__version__,
                  "method": "local HTTP + WebSocket protocol and MuJoCo physics; no browser raster verification",
                  "passed": True, "checks": checks, "last_scene": app.report()}
        target = ROOT / "docs/scene_validation.json"
        target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"passed": True, "checks": checks}, indent=2))
    finally:
        server.stop()


if __name__ == "__main__":
    main()
