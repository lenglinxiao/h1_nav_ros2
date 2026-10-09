from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class TestFastLioLogging(unittest.TestCase):
    def test_missing_log_directory_does_not_reach_null_file_operations(self):
        source = (ROOT / "src/FAST_LIO_ROS2/src/laserMapping.cpp").read_text(encoding="utf-8")

        self.assertIn("if (fp == nullptr)\n        return;", source)
        self.assertIn("if (fp != nullptr)\n            fclose(fp);", source)
        self.assertIn("if (fp2 != nullptr)", source)
        self.assertIn("fclose(fp2);", source)


if __name__ == "__main__":
    unittest.main()
