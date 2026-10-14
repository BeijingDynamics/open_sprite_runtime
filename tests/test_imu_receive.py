import importlib.util
from pathlib import Path
import unittest

from open_sprite_runtime.yahboom_imu import YahboomRawImu, YahboomQuaternion, YahboomStreamDecoder

# Exercise the staged candidate without changing the installed runtime.
candidate = Path(__file__).with_name('imu_receive.py')
if candidate.exists():
    spec = importlib.util.spec_from_file_location('open_sprite_runtime.imu_receive_candidate', candidate)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
else:
    from open_sprite_runtime import imu_receive as module


def frame(function, size):
    data = bytearray([0x7e,0x23,size+5,function]+[0]*size)
    return bytes(data+bytes([sum(data)&255]))


class Port:
    def __init__(self, data=b''):
        self.data = data
        self.reads = 0
    @property
    def in_waiting(self):
        return len(self.data)
    def read(self, count):
        self.reads += 1
        out,self.data = self.data[:count],self.data[count:]
        return out


class ReceiveTests(unittest.TestCase):
    def test_no_wait_when_empty(self):
        p = Port()
        module.drain_imu(p,YahboomStreamDecoder(),lambda *x: self.fail(),clock_ns=lambda:100)
        self.assertEqual(p.reads,0)

    def test_arrival_after_empty_check_is_next_event_not_fault(self):
        class RacingPort(Port):
            checks=0
            @property
            def in_waiting(self):
                self.checks+=1
                return 0 if self.checks==1 else 1
        p=RacingPort()
        module.drain_imu(p,YahboomStreamDecoder(),lambda *x:self.fail(),clock_ns=lambda:100)
        self.assertEqual(p.checks,1)

    def test_drain_both_before_policy_snapshot(self):
        result=[]
        module.drain_imu(Port(frame(4,18)+frame(0x16,16)),YahboomStreamDecoder(),
                         lambda p,t:result.append((p,t)),clock_ns=lambda:100)
        self.assertEqual([type(x[0]) for x in result],[YahboomRawImu,YahboomQuaternion])
        self.assertEqual([x[1] for x in result],[100,100])

    def test_partial_frame_across_reads(self):
        f=frame(4,18); dec=YahboomStreamDecoder(); result=[]
        for b in [f[:8],f[8:]]:
            module.drain_imu(Port(b),dec,lambda p,t:result.append(p),clock_ns=lambda:100)
        self.assertEqual(len(result),1)

    def test_large_backlog_not_retTimestamped(self):
        with self.assertRaisesRegex(RuntimeError,'byte_limit=True.*pending_bytes=257'):
            module.drain_imu(Port(b'x'*257),YahboomStreamDecoder(),lambda *x:self.fail(),clock_ns=lambda:100)

    def test_expired_drain_is_not_committed(self):
        times = iter([100, 30_000_101])
        p = Port(frame(4,18))
        with self.assertRaisesRegex(RuntimeError,'byte_limit=False time_limit=True'):
            module.drain_imu(p,YahboomStreamDecoder(),lambda *x:self.fail(),clock_ns=lambda:next(times,30_000_101))
        self.assertEqual(p.reads,0)

    def test_scheduling_delay_is_diagnostic_and_keeps_batch_start(self):
        for delay in (4_816_000, 8_608_000):
            times = iter([100])
            results=[]
            dec=YahboomStreamDecoder()
            module.drain_imu(Port(frame(4,18)+frame(0x16,16)),dec,
                             lambda p,t:results.append(t),
                             clock_ns=lambda:next(times,100+delay))
            self.assertEqual(results,[100,100])
            self.assertTrue(dec.last_drain_timing['over_2ms_diagnostic_only'])
            with self.assertRaisesRegex(RuntimeError,'stale'):
                module.require_fresh_imu(*results,30_000_101)

    def test_slow_final_read_cannot_commit_stale_frame(self):
        now=[100]
        class SlowPort(Port):
            def read(self,count):
                result=super().read(count)
                now[0]+=31_000_000
                return result
        with self.assertRaisesRegex(RuntimeError,'batch stale'):
            module.drain_imu(SlowPort(frame(4,18)),YahboomStreamDecoder(),
                             lambda *x:self.fail(),clock_ns=lambda:now[0])

    def test_stale_partial_frame_is_not_refreshed(self):
        f=frame(4,18); dec=YahboomStreamDecoder()
        module.drain_imu(Port(f[:8]),dec,lambda *x:self.fail(),clock_ns=lambda:100)
        with self.assertRaisesRegex(RuntimeError,'partial frame stale'):
            module.drain_imu(Port(f[8:]),dec,lambda *x:self.fail(),clock_ns=lambda:31_000_100)

    def test_continuous_split_frames_keep_individual_age(self):
        f=frame(4,18); dec=YahboomStreamDecoder(); results=[]
        module.drain_imu(Port(f[:8]),dec,lambda p,t:results.append(t),clock_ns=lambda:100)
        for k in range(1,6):
            module.drain_imu(Port(f[8:]+f[:8]),dec,lambda p,t:results.append(t),
                             clock_ns=lambda:100+k*10_000_000)
        self.assertEqual(results,[100+k*10_000_000 for k in range(5)])

    def test_three_cycles_reject_without_committing(self):
        with self.assertRaisesRegex(RuntimeError,'report cycles'):
            module.drain_imu(Port(frame(4,18)*3),YahboomStreamDecoder(),lambda *x:self.fail(),clock_ns=lambda:100)

    def test_fresh_and_boundary(self):
        module.require_fresh_imu(1,10_000_001,30_000_001)

    def test_stopped_raw_or_quaternion(self):
        for raw,quat in [(1,30_000_002),(30_000_002,1)]:
            with self.assertRaisesRegex(RuntimeError,'stale'):
                module.require_fresh_imu(raw,quat,30_000_002)

    def test_future_or_missing_timestamp(self):
        for stamp in [0,101]:
            with self.assertRaisesRegex(RuntimeError,'invalid'):
                module.require_fresh_imu(stamp,100,100)

    def test_inference_delay_rechecked(self):
        module.require_fresh_imu(1,1,20_000_001)
        with self.assertRaisesRegex(RuntimeError,'stale'):
            module.require_fresh_imu(1,1,31_000_001)


if __name__=='__main__':
    unittest.main()
