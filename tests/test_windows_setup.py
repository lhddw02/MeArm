"""Check that setup cannot silently reuse a base Python or remove user folders."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.setup_windows import ROOT, child_environment, prepare_environment


class WindowsSetupTests(unittest.TestCase):
    def setUp(self):
        (ROOT / "tmp").mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "tmp")
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    def test_existing_non_environment_folder_is_preserved(self):
        marker = self.folder / "my-file.txt"
        marker.write_text("keep me")
        with self.assertRaisesRegex(ValueError, "Nothing was deleted"):
            prepare_environment(self.folder)
        self.assertEqual(marker.read_text(), "keep me")

    def test_skip_install_without_environment_creates_nothing(self):
        target = self.folder / "not created"
        with self.assertRaisesRegex(ValueError, "existing, installed"):
            prepare_environment(target, skip_install=True)
        self.assertFalse(target.exists())

    def test_base_python_in_environment_directory_is_rejected(self):
        executable = self.folder / "Scripts/python.exe"
        executable.parent.mkdir()
        executable.write_bytes(b"not executed")
        probe = {"prefix": str(self.folder), "base_prefix": str(self.folder),
                 "version": [3, 12], "bits": 64, "implementation": "cpython"}
        reply = subprocess.CompletedProcess([], 0, stdout=json.dumps(probe), stderr="")
        with patch("scripts.setup_windows.subprocess.run", return_value=reply):
            with self.assertRaisesRegex(ValueError, "not a supported local"):
                prepare_environment(self.folder)
        self.assertTrue(executable.exists())

    def test_inherited_package_paths_are_removed_only_for_children(self):
        with patch.dict(os.environ, {"PYTHONHOME": "other-machine", "PYTHONPATH": "wrong-abi/.deps"}):
            child = child_environment()
            self.assertNotIn("PYTHONHOME", child)
            self.assertNotIn("PYTHONPATH", child)
            self.assertEqual(child["PYTHONNOUSERSITE"], "1")
            self.assertEqual(os.environ["PYTHONHOME"], "other-machine")
            self.assertEqual(os.environ["PYTHONPATH"], "wrong-abi/.deps")


if __name__ == "__main__":
    unittest.main()
