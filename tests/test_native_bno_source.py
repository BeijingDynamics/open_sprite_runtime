import time
import socket
import struct
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
from open_sprite_runtime.native_bno_source import NativeBnoSource
from open_sprite_runtime.bno055 import Sample


class SourceTests(unittest.TestCase):
    def test_path_required(self):
        with self.assertRaises(ValueError):NativeBnoSource(None)

    def test_timestamp_and_duplicate(self):
        source=NativeBnoSource('/unused')
        source.latest=Sample(1_000_000_000,1_001_000_000,(0,0,0),(0,0,0),(1,0,0,0),(0,0,9.81),20,{},'')
        received=[]
        with patch('time.monotonic_ns',return_value=1_010_000_000):
            source.drain(lambda p,t:received.append(t))
            source.drain(lambda p,t:received.append(t))
        self.assertEqual(received,[1_000_000_000]*2)
        with patch('time.monotonic_ns',return_value=1_031_000_000):
            with self.assertRaisesRegex(RuntimeError,'stale'):source.drain(lambda *a:None)

    def test_error_latches(self):
        source=NativeBnoSource('/unused');source.error='producer exited'
        with self.assertRaisesRegex(RuntimeError,'producer exited'):source.drain(lambda *a:None)

    @unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'requires Unix sockets')
    def test_missing_producer_cleanup(self):
        source=NativeBnoSource('/nonexistent-sprite-imu-test.sock')
        with self.assertRaises(OSError):source.__enter__()
        self.assertEqual(source.client.fileno(),-1)


@unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'requires Unix sockets')
class IpcFaultTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='sprite_imu_test_')
        self.path = str(Path(self.directory.name) / 'producer.sock')
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.server.bind(self.path)
        self.server.settimeout(1)
        self.source = NativeBnoSource(self.path).__enter__()

    def tearDown(self):
        self.source.__exit__(None, None, None)
        self.server.close()
        self.directory.cleanup()

    def reply(self, sequence=10, age_ns=0, malformed=False):
        request, client = self.server.recvfrom(64)
        self.assertEqual(request, b'GETIMU01')
        now = time.monotonic_ns() - age_ns
        payload = bytearray(192)
        payload[:8] = b'SPIMU001'
        struct.pack_into('<QQQ', payload, 8, sequence, now, now)
        struct.pack_into('<h', payload, 56, 16384)
        struct.pack_into('<13d', payload, 80, 0,0,0,0,0,0,1,0,0,0,0,0,0)
        self.server.sendto(b'bad' if malformed else payload, client)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 1
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertTrue(predicate(), self.source.report())

    def assert_failed(self):
        self.wait_for(lambda: self.source.error is not None)
        with self.assertRaisesRegex(RuntimeError, 'native IMU failed'):
            self.source.drain(lambda *args: None)

    def test_producer_timeout(self):
        self.assert_failed()

    def test_malformed_packet(self):
        self.reply(malformed=True)
        self.assert_failed()

    def test_stale_packet(self):
        self.reply(age_ns=40_000_000)
        self.assert_failed()

    def test_sequence_restart_rejected(self):
        self.reply(sequence=10)
        self.wait_for(lambda: self.source.received == 1)
        self.reply(sequence=1)
        self.assert_failed()

    def test_producer_exit_latches(self):
        self.reply()
        self.wait_for(lambda: self.source.received == 1)
        self.server.close()
        self.assert_failed()
        error = self.source.error
        time.sleep(.03)
        self.assertEqual(self.source.error, error)


if __name__=='__main__':unittest.main()
