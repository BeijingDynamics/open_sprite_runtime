"""Bounded nonblocking IMU reception for the native policy event loop."""
import time

from .yahboom_imu import YahboomQuaternion, YahboomRawImu


def drain_imu(port, decoder, accept, *, clock_ns=time.monotonic_ns):
    """Drain currently available bytes, without waiting for another IMU frame.

    Reject large queues instead of marking buffered old packets as fresh.
    Receipt times are host read timestamps, not device sampling timestamps.
    Only commit decoded packets after the bounded batch passes validation.
    """
    started = clock_ns()
    partial_since = getattr(decoder, '_partial_received_ns', None)
    if decoder.buffer and partial_since is not None and started-partial_since > 30_000_000:
        raise RuntimeError('IMU partial frame stale')
    total = 0
    packets = []
    counts = {YahboomRawImu: 0, YahboomQuaternion: 0}
    for _ in range(8):
        pending = port.in_waiting
        if not pending:
            break
        if total + pending > 256 or clock_ns()-started > 2_000_000:
            raise RuntimeError('IMU receive backlog exceeds bounded drain')
        received_ns = clock_ns()
        payload = port.read(pending)
        if not payload:
            break
        total += len(payload)
        packet_time = min(received_ns, partial_since) if partial_since is not None else received_ns
        decoded = decoder.feed(payload)
        partial_since = (received_ns if decoded else packet_time) if decoder.buffer else None
        decoder._partial_received_ns = partial_since
        for index, packet in enumerate(decoded):
            if type(packet) in counts:
                counts[type(packet)] += 1
                if counts[type(packet)] > 2:
                    raise RuntimeError('IMU backlog contains more than two report cycles')
            packets.append((packet, packet_time if index == 0 else received_ns))
    else:
        raise RuntimeError('IMU receive queue did not drain')
    for packet, received_ns in packets:
        accept(packet, received_ns)


def require_fresh_imu(raw_ns, quaternion_ns, now_ns, maximum_age_ns=30_000_000):
    for name, timestamp in [('raw', raw_ns), ('quaternion', quaternion_ns)]:
        if timestamp <= 0 or timestamp > now_ns:
            raise RuntimeError(f'IMU {name} receipt timestamp invalid')
        if now_ns-timestamp > maximum_age_ns:
            raise RuntimeError(f'IMU {name} stale: {(now_ns-timestamp)/1e6:.3f}ms')
