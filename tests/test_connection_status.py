"""Native behavior/sanitizer tests plus fail-closed firmware ABI checks.
Run: python3 -m unittest discover -s tests -v
The stock image is optional for native tests; build_cfw.sh downloads it.
"""
import importlib.util
import json
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ConnectionStatusTests(unittest.TestCase):
    def test_native_behavior_with_sanitizers(self):
        if not shutil.which("clang"):
            self.skipTest("clang unavailable")
        with tempfile.TemporaryDirectory() as temp:
            executable = str(Path(temp) / "host")
            subprocess.run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(ROOT / "tests/connection_status_host.c"), "-o", executable], check=True)
            env = dict(os.environ, UBSAN_OPTIONS="halt_on_error=1")
            result = subprocess.run([executable], env=env, check=True, capture_output=True, text=True)
            self.assertIn("host contracts: PASS", result.stdout)
            self.assertEqual(result.stderr, "")

    def test_pinned_stock_and_corruptions(self):
        path = ROOT / "g2_2.2.9.22.bin"
        if not path.exists():
            self.skipTest("run build_cfw.sh to obtain the stock image")
        spec = importlib.util.spec_from_file_location("patch_compress", ROOT / "patches/patch_compress.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        stock = path.read_bytes()
        module.validate_connection_status_stock(stock)
        abi = json.loads((ROOT / "patches/connection_status_abi.json").read_text())
        for guard in abi["guards"]:
            damaged = bytearray(stock)
            damaged[module.g2f(int(guard["address"], 0))] ^= 1
            with self.subTest(guard=guard["name"]), self.assertRaisesRegex(ValueError, "ABI mismatch"):
                module.validate_connection_status_stock(damaged)
        damaged = bytearray(stock); damaged[0] ^= 1
        with self.assertRaisesRegex(ValueError, "pinned stock"):
            module.validate_connection_status_stock(damaged)

    def test_reserved_state_does_not_overlap_context_anchors(self):
        abi = json.loads((ROOT / "patches/connection_status_abi.json").read_text())
        address = int(abi["reserved_state"]["address"], 0)
        self.assertGreaterEqual(address, 0x2029f4a8 + 8)
        self.assertLessEqual(address + abi["reserved_state"]["size"], 0x2029f8a8)

if __name__ == "__main__":
    unittest.main()
