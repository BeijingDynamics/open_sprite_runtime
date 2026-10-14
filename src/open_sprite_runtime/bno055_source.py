"""Latest-only BNO055 source; host read time is not sensor capture time."""
import threading
import time

from .bno055 import BNO055, I2CRegisters, CONFIRM
from .imu_receive import require_fresh_imu
from .yahboom_imu import YahboomRawImu, YahboomQuaternion


def sample_packets(sample):
    # P0 output axes physically match the existing Yahboom mounting axes.
    # Reuse SI value containers only; no serial encoding or second remapping.
    return (YahboomRawImu(sample.acceleration_p0_m_s2, sample.gyro_p0_rad_s,
                         (0., 0., 0.)),
            YahboomQuaternion(sample.quaternion_p0_to_world_wxyz))


class Bno055Source:
    def __init__(self, bus=7, address=0x28, initialize=False, io_factory=I2CRegisters):
        self.bus, self.address, self.initialize = bus, address, initialize
        self.io_factory = io_factory
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.delivered_ns = 0
        self.poll_count = 0
        self.skipped_slots = 0
        self.max_read_ms = 0.
        self.thread = None

    def __enter__(self):
        # Complete initialization before connecting to the native controller.
        self.io = self.io_factory(self.bus, self.address)
        try:
            self.sensor = BNO055(self.io)
            cfg = self.sensor.inspect()
            if not cfg['matches'] and self.initialize:
                cfg = self.sensor.configure(CONFIRM)
            self._check(cfg)
            self.thread = threading.Thread(target=self._worker, daemon=True,
                                           name='bno055-100hz')
            self.thread.start()
            return self
        except BaseException:
            self.io.close()
            raise

    @staticmethod
    def _check(cfg):
        if not cfg['matches'] or cfg['system_error'] or cfg['system_status'] != 5:
            raise RuntimeError(f'BNO055 IMUPLUS/P0 not ready: {cfg}')

    def _worker(self):
        deadline = time.monotonic_ns()
        next_check = deadline + 1_000_000_000
        try:
            while not self.stop.is_set():
                sample = self.sensor.read_sample()
                if sample.read_end_ns >= next_check:
                    self._check(self.sensor.inspect())
                    next_check = time.monotonic_ns() + 1_000_000_000
                with self.lock:
                    self.latest = sample
                    self.poll_count += 1
                    self.max_read_ms = max(self.max_read_ms,
                        (sample.read_end_ns-sample.read_start_ns)/1e6)
                deadline += 10_000_000
                now = time.monotonic_ns()
                if now > deadline:
                    skipped = (now-deadline)//10_000_000+1
                    self.skipped_slots += int(skipped)
                    deadline += skipped*10_000_000
                self.stop.wait(max(0, (deadline-time.monotonic_ns())/1e9))
        except Exception as exc:
            with self.lock:
                self.error = f'{type(exc).__name__}: {exc}'
        finally:
            self.io.close()

    def drain(self, accept):
        with self.lock:
            error, sample = self.error, self.latest
        if error:
            raise RuntimeError(f'BNO055 acquisition failed: {error}')
        if sample is None:
            return
        stamp = sample.read_start_ns
        require_fresh_imu(stamp, stamp, time.monotonic_ns())
        if stamp != self.delivered_ns:
            for packet in sample_packets(sample):
                accept(packet, stamp)
            self.delivered_ns = stamp

    def report(self):
        return dict(source='bno055', mount_basis='p0-output-axes',
                    host_poll_hz=100, poll_count=self.poll_count,
                    skipped_slots=self.skipped_slots, max_read_ms=self.max_read_ms,
                    timestamp_basis='host read start; not device sampling timestamp',
                    magnetic_field_available=False, error=self.error)

    def __exit__(self, *args):
        self.stop.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError('BNO055 worker did not stop within 3s')
