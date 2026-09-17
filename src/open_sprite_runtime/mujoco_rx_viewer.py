"""Kinematic MuJoCo display for receive-only physical-motor feedback."""

from __future__ import annotations

from contextlib import AbstractContextManager
import time
from typing import Any, Mapping

import numpy as np

from .damiao import DamiaoFeedback
from .hardware import DIFFERENTIAL_PAIRS
from .motor_mapping import SpriteMotorMap, motor_map_from_hardware_config


class MujocoRxViewer(AbstractContextManager["MujocoRxViewer"]):
    """Render decoded motor state without simulation or transport capability."""

    def __init__(
        self,
        hardware: Mapping[str, Any],
        policy_joint_names: list[str] | tuple[str, ...],
        mjcf_path: str,
        *,
        root_height_m: float = 0.52,
        refresh_hz: float = 50.0,
    ) -> None:
        import mujoco
        import mujoco.viewer

        if refresh_hz <= 0.0:
            raise ValueError("refresh_hz must be positive")
        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(mjcf_path)
        self._data = mujoco.MjData(self._model)
        self._mapping: SpriteMotorMap = motor_map_from_hardware_config(
            hardware, policy_joint_names, require_armable=False
        )
        self._joint_names = tuple(policy_joint_names)
        self._qpos_addresses: dict[str, int] = {}
        for name in self._joint_names:
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"MuJoCo model is missing policy joint {name}")
            self._qpos_addresses[name] = int(self._model.jnt_qposadr[joint_id])

        free_joints = np.flatnonzero(self._model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        if len(free_joints) != 1:
            raise ValueError("MuJoCo viewer model must contain exactly one free joint")
        root_qpos = int(self._model.jnt_qposadr[int(free_joints[0])])
        self._data.qpos[root_qpos : root_qpos + 3] = (0.0, 0.0, root_height_m)
        self._data.qpos[root_qpos + 3 : root_qpos + 7] = (1.0, 0.0, 0.0, 0.0)

        neutral = np.zeros(31, dtype=np.float64)
        self._motor_positions = self._mapping.joint_to_motor_positions(neutral)
        self._frozen_joints = {
            joint
            for pair_name, joints in DIFFERENTIAL_PAIRS.items()
            if not bool(hardware["differentials"][pair_name].get("calibrated"))
            for joint in joints
        }
        self._minimum_sync_interval_s = 1.0 / refresh_hz
        self._last_sync = 0.0
        mujoco.mj_forward(self._model, self._data)
        self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
        self._viewer.cam.lookat[:] = (0.0, 0.0, 0.45)
        self._viewer.cam.distance = 1.8
        self._viewer.cam.azimuth = 90.0
        self._viewer.cam.elevation = -8.0
        self._viewer.sync()

    @property
    def frozen_joints(self) -> tuple[str, ...]:
        return tuple(sorted(self._frozen_joints))

    def is_running(self) -> bool:
        return bool(self._viewer.is_running())

    def update(self, feedback: DamiaoFeedback) -> None:
        if feedback.motor_name not in self._motor_positions:
            raise ValueError(f"unexpected motor feedback {feedback.motor_name}")
        self._motor_positions[feedback.motor_name] = feedback.position_rad
        now = time.monotonic()
        if now - self._last_sync < self._minimum_sync_interval_s:
            return
        positions = self._mapping.motor_to_joint_positions(self._motor_positions)
        for index, name in enumerate(self._joint_names):
            self._data.qpos[self._qpos_addresses[name]] = (
                0.0 if name in self._frozen_joints else positions[index]
            )
        self._mujoco.mj_forward(self._model, self._data)
        self._viewer.sync()
        self._last_sync = now

    def close(self) -> None:
        self._viewer.close()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
