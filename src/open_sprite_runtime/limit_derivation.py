"""Derive reviewable motor-limit candidates from the frozen URDF joint limits."""

from __future__ import annotations

import hashlib
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .contracts import PolicyContract
from .motor_mapping import motor_map_from_hardware_config


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _urdf_joint_limits(path: Path, joint_names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    result: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in joint_names:
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"URDF joint {name} has no finite lower/upper limit")
        low, high = float(limit.get("lower")), float(limit.get("upper"))
        if not np.isfinite((low, high)).all() or low >= high:
            raise ValueError(f"URDF joint {name} has invalid lower/upper limit")
        result[name] = (low, high)
    missing = sorted(set(joint_names) - set(result))
    if missing:
        raise ValueError("URDF is missing policy joint limits: " + ", ".join(missing))
    return result


def _inset(bounds: tuple[float, float], margin_rad: float) -> tuple[float, float]:
    low, high = bounds
    if not np.isfinite(margin_rad) or margin_rad <= 0.0:
        raise ValueError("soft margin must be finite and positive")
    if 2.0 * margin_rad >= high - low:
        raise ValueError(f"soft margin {margin_rad} leaves no usable range inside {bounds}")
    return low + margin_rad, high - margin_rad


def _bounds(values) -> list[float]:
    array = np.asarray(tuple(values), dtype=np.float64)
    return [float(np.min(array)), float(np.max(array))]


def derive_limit_candidates(
    hardware: dict,
    contract: PolicyContract,
    urdf_path: str | Path,
    *,
    soft_margin_rad: float = 0.05,
) -> dict:
    urdf = Path(urdf_path).expanduser().resolve()
    actual_hash = _sha256(urdf)
    expected_hash = str(contract.data.get("asset_urdf_sha256") or "")
    if not expected_hash or actual_hash != expected_hash:
        raise ValueError(
            f"URDF SHA256 mismatch: expected {expected_hash or '<missing>'}, got {actual_hash}"
        )
    joint_names = tuple(contract.data["joint_names"])
    hard_joint = _urdf_joint_limits(urdf, joint_names)
    soft_joint = {name: _inset(bounds, soft_margin_rad) for name, bounds in hard_joint.items()}
    mapping = motor_map_from_hardware_config(hardware, joint_names, require_armable=False)
    motors: dict[str, dict] = {}

    for joint_name, direct in mapping.direct.items():
        hard = _bounds(direct.joint_to_drive_position(value) for value in hard_joint[joint_name])
        soft = _bounds(direct.joint_to_drive_position(value) for value in soft_joint[joint_name])
        motors[direct.motor_name] = {
            "topology": "direct",
            "policy_joints": [joint_name],
            "hard_limit_rad_candidate": hard,
            "soft_limit_rad_candidate": soft,
        }

    for pair_name, pair in mapping.differentials.items():
        hard_corners = itertools.product(*(hard_joint[name] for name in pair.joint_names))
        soft_corners = itertools.product(*(soft_joint[name] for name in pair.joint_names))
        hard_positions = np.asarray(
            [pair.joint_to_drive_position(corner) for corner in hard_corners], dtype=np.float64
        )
        soft_positions = np.asarray(
            [pair.joint_to_drive_position(corner) for corner in soft_corners], dtype=np.float64
        )
        for index, motor_name in enumerate(pair.motor_names):
            motors[motor_name] = {
                "topology": "differential",
                "differential_pair": pair_name,
                "policy_joints": list(pair.joint_names),
                "hard_limit_rad_candidate": _bounds(hard_positions[:, index]),
                "soft_limit_rad_candidate": _bounds(soft_positions[:, index]),
            }

    if set(motors) != set(mapping.physical_motor_names):
        raise RuntimeError("derived motor limits do not exactly cover the 31-motor map")
    return {
        "mode": "offline_urdf_limit_derivation_no_hardware_io",
        "asset_urdf": str(contract.data.get("asset_urdf") or urdf.name),
        "asset_urdf_sha256": actual_hash,
        "soft_margin_rad_each_joint_boundary": soft_margin_rad,
        "joint_limits": {
            name: {
                "hard_limit_rad_candidate": list(hard_joint[name]),
                "soft_limit_rad_candidate": list(soft_joint[name]),
            }
            for name in joint_names
        },
        "motor_limits": motors,
        "differential_pairs": {
            name: {
                "joint_order": list(pair.joint_names),
                "motor_order": list(pair.motor_names),
                "joint_to_drive_matrix": pair.joint_to_drive_matrix.tolist(),
            }
            for name, pair in mapping.differentials.items()
        },
        "confirmed_physical_hard_limits": False,
        "warnings": [
            "URDF limits are design candidates until confirmed against physical hard stops.",
            "Differential motor min/max boxes do not replace joint-space limit checks.",
            "Runtime must reject joint-space violations before mapping to differential motors.",
        ],
    }
