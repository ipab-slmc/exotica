import subprocess
import unittest


class TestUnknownInitializerTypes(unittest.TestCase):
    def test_no_unknown(self):
        result = subprocess.run(
            ["python3", "-c", "import pyexotica as exo; exo.Setup.get_initializers()"],
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        self.assertNotIn(b"Skipping", result.stdout)


if __name__ == "__main__":
    unittest.main()
