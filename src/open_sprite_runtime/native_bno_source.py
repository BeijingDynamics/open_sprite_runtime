"""Opt-in native producer adapter. Errors latch; no restart or timestamp refresh."""
from pathlib import Path
import socket
import tempfile
import threading
import time
import os
from .bno055_source import sample_packets
from .imu_receive import require_fresh_imu
from .native_imu_packet import decode_packet, decode_software_packet


class NativeBnoSource:
    def __init__(self, path):
        if not path:
            raise ValueError('native IMU socket required')
        self.path=path
        self.protocol=os.environ.get('SPRITE_NATIVE_IMU_PROTOCOL','hardware-v1')
        if self.protocol not in ('hardware-v1','software-v2'):
            raise ValueError('unknown native IMU protocol')
        self.stop=threading.Event()
        self.lock=threading.Lock()
        self.latest=None
        self.error=None
        self.received=0
        self.delivered=0
        self.sequence=0
        self.max_age_ms=0.0
        self.thread=None
        self.client=None
        self.directory=None

    def __enter__(self):
        self.directory=tempfile.TemporaryDirectory(prefix='sprite_native_imu_')
        try:
            self.client=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            self.client.bind(str(Path(self.directory.name)/'rx.sock'))
            self.client.connect(self.path)
            self.client.settimeout(.025)
            self.thread=threading.Thread(target=self._worker,name='native-imu-ipc',daemon=True)
            self.thread.start()
            return self
        except BaseException:
            if self.client is not None:self.client.close()
            self.directory.cleanup()
            raise

    def _worker(self):
        try:
            while not self.stop.is_set():
                begin=time.monotonic()
                self.client.send(b'GETIMU01')
                payload=self.client.recv(193)
                now=time.monotonic_ns()
                decoder=decode_software_packet if self.protocol=='software-v2' else decode_packet
                value=decoder(payload,now,self.sequence)
                with self.lock:
                    self.latest=value.sample
                    self.sequence=value.sequence
                    self.received+=1
                    self.max_age_ms=max(self.max_age_ms,(now-value.sample.read_start_ns)/1e6)
                self.stop.wait(max(0,.005-(time.monotonic()-begin)))
        except Exception as exc:
            if not self.stop.is_set():
                with self.lock:self.error=f'{type(exc).__name__}: {exc}'

    def drain(self, accept):
        with self.lock:
            error,sample=self.error,self.latest
        if error:raise RuntimeError(f'native IMU failed: {error}')
        if sample is None:return
        stamp=sample.read_start_ns
        require_fresh_imu(stamp,stamp,time.monotonic_ns())
        if stamp!=self.delivered:
            for packet in sample_packets(sample):accept(packet,stamp)
            self.delivered=stamp

    def report(self):
        with self.lock:
            return dict(source='native-bno055',socket=self.path,received=self.received,
                protocol=self.protocol,
                last_sequence=self.sequence,max_age_at_receive_ms=self.max_age_ms,
                timestamp_basis='native host read start, not sensor capture time',error=self.error)

    def __exit__(self,*args):
        self.stop.set()
        if self.thread is not None:self.thread.join(1)
        if self.client is not None:self.client.close()
        if self.directory is not None:self.directory.cleanup()
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('native IMU adapter shutdown timeout')
