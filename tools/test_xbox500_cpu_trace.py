"""No hardware: test timer-vs-runqueue attribution and unmatched trace handling."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock
from analyze_xbox500_cpu_trace import analyze
import run_xbox500_cpu_audit as audit


class TraceTest(unittest.TestCase):
    def test_qos_released_when_trace_setup_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'reports').mkdir()
            trace = MagicMock()
            trace.__enter__.side_effect = RuntimeError('simulated trace setup failure')
            with patch.object(audit,'ROOT',root), patch.object(audit,'snapshot',return_value={}), \
                 patch.object(audit,'Trace',return_value=trace), \
                 patch.object(audit.os,'open',return_value=123) as opened, \
                 patch.object(audit.os,'write',return_value=4), \
                 patch.object(audit.os,'close') as closed, \
                 patch.object(audit.subprocess,'Popen') as launch:
                with self.assertRaisesRegex(RuntimeError,'simulated'):
                    audit.phase('B',root/'phase',True)
                opened.assert_called_once()
                closed.assert_called_once_with(123)
                launch.assert_not_called()

    def test_timer_and_scheduler_delays(self):
        lines = [
            'sprite_can_xbox-100 [005] d..2 0.999000: hrtimer_start: hrtimer=0xabc function=hrtimer_wakeup expires=1000000000 softexpires=1000000000 mode=ABS',
            '<idle>-0 [005] d.h1 1.000400: hrtimer_expire_entry: hrtimer=0xabc function=hrtimer_wakeup now=1000400000',
            '<idle>-0 [005] d.h2 1.000410: sched_wakeup: comm=sprite_can_xbox pid=100 prio=49 target_cpu=005',
            '<idle>-0 [005] d..2 1.000420: sched_switch: prev_comm=swapper/5 prev_pid=0 ==> next_comm=sprite_can_xbox next_pid=100 next_prio=49',
            'sprite_can_xbox-100 [005] d..2 1.001000: hrtimer_start: hrtimer=0xabc function=hrtimer_wakeup expires=1002000000 softexpires=1002000000 mode=ABS',
            '<idle>-0 [005] d.h1 1.002000: hrtimer_expire_entry: hrtimer=0xabc function=hrtimer_wakeup now=1002000000',
            '<idle>-0 [005] d.h2 1.002010: sched_wakeup: comm=sprite_can_xbox pid=100 prio=49 target_cpu=005',
            'other-200 [005] d..2 1.002210: sched_switch: prev_comm=other prev_pid=200 ==> next_comm=sprite_can_xbox next_pid=100 next_prio=49',
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'trace'
            path.write_text('\n'.join(lines))
            result = analyze(path,{100})
            self.assertEqual(result['matched_cycles'],2)
            self.assertEqual(result['worst_cycles'][0]['timer_lateness_us'],400)
            self.assertEqual(result['worst_cycles'][0]['wake_to_run_us'],10)
            self.assertEqual(result['worst_cycles'][1]['timer_lateness_us'],0)
            self.assertEqual(result['worst_cycles'][1]['wake_to_run_us'],200)
            self.assertFalse(analyze(path,{999})['attribution_available'])


if __name__ == '__main__':
    unittest.main()
