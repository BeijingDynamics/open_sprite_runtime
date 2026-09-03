import unittest

from open_sprite_runtime.timing import TimingMonitor


class TimingMonitorTests(unittest.TestCase):
    def test_reports_jitter_and_overruns(self) -> None:
        monitor = TimingMonitor(
            period_ns=2_000_000,
            maximum_start_jitter_ns=1_000_000,
            maximum_work_duration_ns=1_000_000,
        )
        monitor.record(10_000_000, 10_200_000, 10_500_000)
        monitor.record(12_000_000, 13_500_000, 14_700_000)
        report = monitor.report()
        self.assertEqual(report["samples"], 2)
        self.assertEqual(report["start_jitter_overrun_count"], 1)
        self.assertEqual(report["work_duration_overrun_count"], 1)

    def test_rejects_negative_duration(self) -> None:
        monitor = TimingMonitor(2_000_000, 1_000_000, 1_000_000)
        with self.assertRaisesRegex(ValueError, "precedes"):
            monitor.record(0, 10, 9)


if __name__ == "__main__":
    unittest.main()
