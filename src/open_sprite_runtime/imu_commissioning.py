"""Read-only Yahboom IMU commissioning and MuJoCo pose visualization."""

from __future__ import annotations

from dataclasses import dataclass
import gc
import math
from pathlib import Path
import time

import numpy as np

from .imu import ImuMount, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix
from .yahboom_imu import YahboomQuaternion, YahboomRawImu, YahboomStreamDecoder


def _open_serial(device: str, baud_rate: int):
    if not device.startswith("/dev/"):
        raise ValueError("serial device must be an absolute /dev path")
    if baud_rate <= 0:
        raise ValueError("baud rate must be positive")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required for Yahboom IMU access") from exc
    port = serial.Serial(
        port=device,
        baudrate=baud_rate,
        timeout=0.02,
        write_timeout=0,
        exclusive=True,
    )
    port.dtr = False
    port.rts = False
    port.reset_input_buffer()
    return port


def body_orientation_matrix(
    sensor_to_world_wxyz: tuple[float, float, float, float], mount: ImuMount
) -> np.ndarray:
    """Return world-from-body rotation for the fixed sensor mounting."""
    world_from_sensor = quaternion_wxyz_to_matrix(sensor_to_world_wxyz)
    sensor_from_body = mount.sensor_to_body_matrix.T
    return world_from_sensor @ sensor_from_body


def relative_body_orientation(
    initial_world_from_body: np.ndarray, current_world_from_body: np.ndarray
) -> np.ndarray:
    """Express current body attitude in the initial body/world frame."""
    return np.asarray(initial_world_from_body).T @ np.asarray(current_world_from_body)


def _roll_pitch_yaw(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, yaw


@dataclass(frozen=True)
class StaticImuAuditReport:
    result: dict

    @property
    def passed(self) -> bool:
        return bool(self.result["passed"])

    def to_dict(self) -> dict:
        return dict(self.result)


def collect_static_imu_audit(
    device: str,
    baud_rate: int,
    duration_s: float,
) -> StaticImuAuditReport:
    """Measure a stationary IMU stream without transmitting any serial bytes."""
    if not math.isfinite(duration_s) or not 3.0 <= duration_s <= 120.0:
        raise ValueError("duration must be finite and in [3, 120] seconds")
    mount = ImuMount.sprite0825_rear_pelvis()
    decoder = YahboomStreamDecoder()
    accelerations: list[tuple[float, float, float]] = []
    angular_velocities: list[tuple[float, float, float]] = []
    quaternions: list[tuple[float, float, float, float]] = []
    body_rotations: list[np.ndarray] = []
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(duration_s * 1.0e9)
    with _open_serial(device, baud_rate) as port:
        while time.monotonic_ns() < deadline_ns:
            payload = port.read(max(port.in_waiting, 1))
            if not payload:
                continue
            for packet in decoder.feed(payload):
                if isinstance(packet, YahboomRawImu):
                    accelerations.append(packet.acceleration_m_s2)
                    angular_velocities.append(packet.angular_velocity_rad_s)
                elif isinstance(packet, YahboomQuaternion):
                    quaternions.append(packet.wxyz)
                    body_rotations.append(body_orientation_matrix(packet.wxyz, mount))
    elapsed_s = (time.monotonic_ns() - start_ns) / 1.0e9

    if not accelerations or not angular_velocities or not quaternions:
        raise RuntimeError("stationary audit received an incomplete IMU stream")
    acceleration_sensor = np.asarray(accelerations, dtype=np.float64)
    angular_velocity_sensor = np.asarray(angular_velocities, dtype=np.float64)
    quaternion_values = np.asarray(quaternions, dtype=np.float64)
    acceleration_body = acceleration_sensor @ mount.sensor_to_body_matrix.T
    angular_velocity_body = angular_velocity_sensor @ mount.sensor_to_body_matrix.T
    acceleration_norm = np.linalg.norm(acceleration_sensor, axis=1)
    quaternion_norm = np.linalg.norm(quaternion_values, axis=1)
    projected_gravity = np.asarray(
        [rotation.T @ np.array([0.0, 0.0, -1.0]) for rotation in body_rotations]
    )
    yaw_values = np.unwrap(
        np.asarray([_roll_pitch_yaw(rotation)[2] for rotation in body_rotations])
    )
    yaw_drift_deg = math.degrees(float(yaw_values[-1] - yaw_values[0]))
    yaw_drift_deg_per_min = yaw_drift_deg * 60.0 / elapsed_s
    quaternion_error_max = float(np.max(np.abs(quaternion_norm - 1.0)))
    acceleration_norm_mean = float(np.mean(acceleration_norm))
    gyro_bias_body = np.mean(angular_velocity_body, axis=0)
    gyro_bias_norm = float(np.linalg.norm(gyro_bias_body))
    raw_rate_hz = len(accelerations) / elapsed_s
    quaternion_rate_hz = len(quaternions) / elapsed_s
    upright_gravity = np.mean(projected_gravity, axis=0)
    upright_candidate = bool(
        np.linalg.norm(upright_gravity[:2]) < 0.15 and upright_gravity[2] < -0.95
    )
    passed = bool(
        90.0 <= raw_rate_hz <= 110.0
        and 90.0 <= quaternion_rate_hz <= 110.0
        and decoder.rejected_frame_count == 0
        and quaternion_error_max <= 0.05
        and 8.5 <= acceleration_norm_mean <= 11.2
        and gyro_bias_norm <= 0.05
    )
    result = {
        "mode": "read_only_stationary_yahboom_imu_audit_no_tx",
        "device": device,
        "baud_rate": baud_rate,
        "requested_duration_s": duration_s,
        "elapsed_s": elapsed_s,
        "raw_sample_count": len(accelerations),
        "quaternion_sample_count": len(quaternions),
        "raw_rate_hz": raw_rate_hz,
        "quaternion_rate_hz": quaternion_rate_hz,
        "rejected_frame_count": decoder.rejected_frame_count,
        "acceleration_sensor_mean_m_s2": np.mean(acceleration_sensor, axis=0).tolist(),
        "acceleration_body_mean_m_s2": np.mean(acceleration_body, axis=0).tolist(),
        "acceleration_norm_mean_m_s2": acceleration_norm_mean,
        "acceleration_norm_std_m_s2": float(np.std(acceleration_norm)),
        "angular_velocity_body_bias_rad_s": gyro_bias_body.tolist(),
        "angular_velocity_body_bias_norm_rad_s": gyro_bias_norm,
        "angular_velocity_body_rms_rad_s": np.sqrt(
            np.mean(np.square(angular_velocity_body), axis=0)
        ).tolist(),
        "quaternion_norm_error_max": quaternion_error_max,
        "projected_gravity_body_mean": upright_gravity.tolist(),
        "projected_gravity_body_std": np.std(projected_gravity, axis=0).tolist(),
        "yaw_drift_deg": yaw_drift_deg,
        "yaw_drift_deg_per_min": yaw_drift_deg_per_min,
        "upright_mount_candidate": upright_candidate,
        "passed": passed,
    }
    return StaticImuAuditReport(result)


def run_mujoco_imu_viewer(
    device: str,
    baud_rate: int,
    duration_s: float,
    mjcf_path: str,
    *,
    root_height_m: float = 0.52,
    refresh_hz: float = 50.0,
    orientation_mode: str = "absolute",
) -> dict:
    """Drive only the MuJoCo floating-base attitude from a read-only IMU stream."""
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= 600.0:
        raise ValueError("duration must be finite and in (0, 600] seconds")
    if not math.isfinite(refresh_hz) or not 1.0 <= refresh_hz <= 100.0:
        raise ValueError("refresh_hz must be finite and in [1, 100]")
    if orientation_mode not in {"absolute", "relative"}:
        raise ValueError("orientation_mode must be 'absolute' or 'relative'")
    path = Path(mjcf_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    free_joints = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    if len(free_joints) != 1:
        raise ValueError("MuJoCo IMU viewer requires exactly one free joint")
    root_qpos = int(model.jnt_qposadr[int(free_joints[0])])
    data.qpos[root_qpos : root_qpos + 3] = (0.0, 0.0, root_height_m)
    model_neutral_rotation = quaternion_wxyz_to_matrix(
        tuple(data.qpos[root_qpos + 3 : root_qpos + 7])
    )
    mount = ImuMount.sprite0825_rear_pelvis()
    decoder = YahboomStreamDecoder()
    initial_world_from_body: np.ndarray | None = None
    quaternion_count = 0
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(duration_s * 1.0e9)
    minimum_sync_s = 1.0 / refresh_hz
    last_sync = 0.0

    with _open_serial(device, baud_rate) as port:
        viewer = mujoco.viewer.launch_passive(model, data)
        try:
            viewer.cam.lookat[:] = (0.0, 0.0, 0.45)
            viewer.cam.distance = 1.8
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -8.0
            while time.monotonic_ns() < deadline_ns and viewer.is_running():
                payload = port.read(max(port.in_waiting, 1))
                if not payload:
                    continue
                for packet in decoder.feed(payload):
                    if not isinstance(packet, YahboomQuaternion):
                        continue
                    world_from_body = body_orientation_matrix(packet.wxyz, mount)
                    if initial_world_from_body is None:
                        initial_world_from_body = world_from_body
                    if orientation_mode == "absolute":
                        display_rotation = world_from_body
                    else:
                        relative = relative_body_orientation(
                            initial_world_from_body, world_from_body
                        )
                        display_rotation = model_neutral_rotation @ relative
                    data.qpos[root_qpos + 3 : root_qpos + 7] = (
                        matrix_to_quaternion_wxyz(display_rotation)
                    )
                    quaternion_count += 1
                now = time.monotonic()
                if now - last_sync >= minimum_sync_s:
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    last_sync = now
        finally:
            # On Jetson/aarch64, deferring the passive viewer wrapper to Python
            # interpreter shutdown can make GLFW teardown segfault.  Destroy it
            # while MuJoCo and GLFW are still fully alive.
            viewer.close()
            del viewer
            gc.collect()
    elapsed_s = (time.monotonic_ns() - start_ns) / 1.0e9
    return {
        "mode": f"read_only_imu_to_mujoco_{orientation_mode}_attitude_no_tx",
        "orientation_mode": orientation_mode,
        "device": device,
        "mjcf": str(path),
        "elapsed_s": elapsed_s,
        "quaternion_count": quaternion_count,
        "quaternion_rate_hz": quaternion_count / elapsed_s,
        "rejected_frame_count": decoder.rejected_frame_count,
        "serial_write_count": 0,
        "can_tx_count": 0,
        "passed": quaternion_count > 0 and decoder.rejected_frame_count == 0,
    }
