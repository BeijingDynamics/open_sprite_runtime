"""Bounded nonblocking IMU reception for the native policy event loop."""
import time

from .yahboom_imu import YahboomQuaternion, YahboomRawImu


def drain_imu(port, decoder, accept, *, clock_ns=time.monotonic_ns):
    started = clock_ns()
    cpu_started = time.thread_time_ns()
    try:
        _drain_imu(port, decoder, accept, started=started, clock_ns=clock_ns)
    finally:
        wall_ms = (clock_ns() - started) / 1e6
        decoder.last_drain_timing = {
            'wall_ms': wall_ms,
            'thread_cpu_ms': (time.thread_time_ns() - cpu_started) / 1e6,
            'over_2ms_diagnostic_only': wall_ms > 2.0,
        }
        stats = getattr(decoder, 'drain_timing_summary', {
            'calls': 0, 'over_2ms_count': 0, 'maximum_wall_ms': 0.0,
            'maximum_thread_cpu_ms': 0.0,
        })
        stats['calls'] += 1
        stats['over_2ms_count'] += int(wall_ms > 2.0)
        stats['maximum_wall_ms'] = max(stats['maximum_wall_ms'], wall_ms)
        stats['maximum_thread_cpu_ms'] = max(
            stats['maximum_thread_cpu_ms'], decoder.last_drain_timing['thread_cpu_ms'])
        decoder.drain_timing_summary = stats


def _drain_imu(port, decoder, accept, *, started, clock_ns):
    """Drain currently available bytes, without waiting for another IMU frame.

    Reject large queues instead of marking buffered old packets as fresh.
    Receipt times are host read timestamps, not device sampling timestamps.
    Only commit decoded packets after the bounded batch passes validation.
    """
    partial_since = getattr(decoder, '_partial_received_ns', None)
    if decoder.buffer and partial_since is not None and started-partial_since > 30_000_000:
        raise RuntimeError('IMU partial frame stale')
    total = 0
    packets = []
    counts = {YahboomRawImu: 0, YahboomQuaternion: 0}
    for iteration in range(8):
        pending = port.in_waiting
        if not pending:
            break
        elapsed_ns = clock_ns()-started
        byte_limit = total + pending > 256
        time_limit = elapsed_ns > 30_000_000
        if byte_limit or time_limit:
            raise RuntimeError(
                'IMU receive backlog exceeds bounded drain: '
                f'byte_limit={byte_limit} time_limit={time_limit} '
                f'read_bytes={total} pending_bytes={pending} '
                f'elapsed_ms={elapsed_ns/1e6:.3f} iteration={iteration} '
                f'raw_frames={counts[YahboomRawImu]} '
                f'quaternion_frames={counts[YahboomQuaternion]}'
            )
        # Conservatively age this entire batch from before the first ioctl.
        # A slow read or descheduling must not make buffered data look newer.
        received_ns = started
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
    if clock_ns() - started > 30_000_000:
        raise RuntimeError('IMU receive batch stale before commit')
    for packet, received_ns in packets:
        if clock_ns() - received_ns > 30_000_000:
            raise RuntimeError('IMU decoded packet stale before commit')
        accept(packet, received_ns)


def require_fresh_imu(raw_ns, quaternion_ns, now_ns, maximum_age_ns=30_000_000):
    for name, timestamp in [('raw', raw_ns), ('quaternion', quaternion_ns)]:
        if timestamp <= 0 or timestamp > now_ns:
            raise RuntimeError(f'IMU {name} receipt timestamp invalid')
        if now_ns-timestamp > maximum_age_ns:
            raise RuntimeError(f'IMU {name} stale: {(now_ns-timestamp)/1e6:.3f}ms')
