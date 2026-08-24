import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE_PATH = Path(__file__).with_name("rl_live_diff.py")
SPEC = importlib.util.spec_from_file_location("rl_live_diff", MODULE_PATH)
runner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(runner)


class DifferentialRunnerTests(unittest.TestCase):
    def test_decimal_range_is_inclusive(self):
        self.assertEqual(runner.parse_number_range("0:0.5:0.25"), [0.0, 0.25, 0.5])

    def test_expands_cartesian_json_path_sweeps(self):
        scenarios = [{"name": "bounce", "ball": {"p": [0, 0, 90], "v": [0, 0, -1]}}]
        expanded = runner.expand_sweeps(
            scenarios, ["ball.p.2=90:91:1", "ball.v.2=-2:-1:1"]
        )
        self.assertEqual(len(expanded), 4)
        self.assertEqual(expanded[0]["ball"]["p"][2], 90.0)
        self.assertEqual(expanded[-1]["ball"]["v"][2], -1.0)

    def test_exact_name_reports_missing_scenario(self):
        with self.assertRaisesRegex(ValueError, "not found"):
            runner.select_scenarios([{"name": "a"}], ["b"], [], None)

    def test_rejects_excessive_cartesian_sweep(self):
        with self.assertRaisesRegex(ValueError, "10,000 variants"):
            runner.expand_sweeps(
                [{"name": "x", "a": 0, "b": 0}],
                ["a=0:100:1", "b=0:100:1"],
            )

    def test_capture_names_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = runner.save_capture(root, {"name": "same"}, {"ok": True})
            second = runner.save_capture(root, {"name": "same"}, {"ok": True})
            self.assertNotEqual(first, second)
            self.assertTrue(first.exists())
            self.assertTrue(second.exists())

    def test_evaluator_runs_from_package_directory(self):
        with patch.object(runner.subprocess, "run") as run:
            run.return_value.stdout = "{}"
            runner.evaluate_capture(Path("evaluator"), Path("capture.json"), 25, False)

        self.assertEqual(run.call_args.kwargs["cwd"], runner.ROOT / "rocketsim")


if __name__ == "__main__":
    unittest.main()
