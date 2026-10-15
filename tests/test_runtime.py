"""Regression checks for optional wheel selection; no simulator imports needed."""
from contextlib import contextmanager, ExitStack
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("mearm_runtime", ROOT / "scripts/_runtime.py")
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


class DependencySelectionTests(unittest.TestCase):
    def setUp(self):
        scratch = ROOT / "tmp"
        scratch.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="runtime-test-", dir=scratch)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.prefix = self.root / "python"
        wheel = self.root / ".deps/example-1.0.dist-info/WHEEL"
        wheel.parent.mkdir(parents=True)
        wheel.write_text(
            "Wheel-Version: 1.0\nRoot-Is-Purelib: false\n"
            "Tag: cp312-cp312-win_amd64\n",
            encoding="utf-8",
        )

    @contextmanager
    def python_environment(self, minor=12):
        with ExitStack() as stack:
            stack.enter_context(mock.patch.dict(runtime.os.environ, {"CONDA_PREFIX": ""}))
            stack.enter_context(mock.patch.object(runtime.sys, "prefix", str(self.prefix)))
            stack.enter_context(mock.patch.object(runtime.sys, "base_prefix", str(self.prefix)))
            stack.enter_context(mock.patch.object(
                runtime.sys, "version_info", SimpleNamespace(major=3, minor=minor)))
            stack.enter_context(mock.patch.object(runtime.sys, "path", ["installed", "stdlib"]))
            stack.enter_context(mock.patch.object(
                runtime.sysconfig, "get_platform", return_value="win-amd64"))
            yield

    def assert_local_wheels_excluded(self):
        before = list(runtime.sys.path)
        self.assertFalse(runtime.configure_dependencies(self.root))
        self.assertEqual(runtime.sys.path, before)

    def test_active_conda_ignores_even_compatible_wheels(self):
        with self.python_environment(), mock.patch.dict(
                runtime.os.environ, {"CONDA_PREFIX": str(self.prefix)}):
            self.assert_local_wheels_excluded()

    def test_conda_metadata_detected_without_environment_variable(self):
        (self.prefix / "conda-meta").mkdir(parents=True)
        with self.python_environment():
            self.assert_local_wheels_excluded()

    def test_virtual_environment_ignores_compatible_wheels(self):
        with self.python_environment(), mock.patch.object(
                runtime.sys, "base_prefix", str(self.root / "base-python")):
            self.assert_local_wheels_excluded()

    def test_python310_rejects_python312_native_wheels(self):
        with self.python_environment(minor=10):
            self.assert_local_wheels_excluded()

    def test_wrong_platform_rejects_native_wheels(self):
        with self.python_environment(), mock.patch.object(
                runtime.sysconfig, "get_platform", return_value="win32"):
            self.assert_local_wheels_excluded()

    def test_matching_wheels_append_after_installed_packages_once(self):
        with self.python_environment():
            self.assertTrue(runtime.configure_dependencies(self.root))
            expected = ["installed", "stdlib", str(self.root / ".deps")]
            self.assertEqual(runtime.sys.path, expected)
            self.assertTrue(runtime.configure_dependencies(self.root))
            self.assertEqual(runtime.sys.path, expected)


if __name__ == "__main__":
    unittest.main()
