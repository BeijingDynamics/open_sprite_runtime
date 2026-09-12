"""Offline, fail-closed validation of SocketCAN receive-only state."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class SocketCanRxPreflightReport:
    expected_interfaces: tuple[str, ...]
    observed_interfaces: tuple[str, ...]
    listen_only_interfaces: tuple[str, ...]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed, "hardware_tx_attempts": 0}


def _ctrlmodes(entry: dict[str, Any]) -> set[str]:
    linkinfo = entry.get("linkinfo")
    info_data = linkinfo.get("info_data") if isinstance(linkinfo, dict) else None
    raw = info_data.get("ctrlmode") if isinstance(info_data, dict) else None
    if isinstance(raw, str):
        return {raw.upper()}
    if isinstance(raw, list):
        return {str(value).upper() for value in raw}
    if isinstance(raw, dict):
        return {str(key).upper() for key, value in raw.items() if value is True}
    return set()


def audit_socketcan_rx_snapshot(
    snapshot: Any, expected_interfaces: Iterable[str]
) -> SocketCanRxPreflightReport:
    """Audit structured output from ``ip -j -d link show`` without opening CAN."""
    expected = tuple(expected_interfaces)
    errors: list[str] = []
    if len(expected) != 4 or len(set(expected)) != 4:
        errors.append("exactly four unique expected interfaces are required")
    if not isinstance(snapshot, list):
        snapshot = []
        errors.append("SocketCAN snapshot must be a JSON array")

    entries = {
        entry.get("ifname"): entry
        for entry in snapshot
        if isinstance(entry, dict) and isinstance(entry.get("ifname"), str)
    }
    listen_only: list[str] = []
    for name in expected:
        entry = entries.get(name)
        if entry is None:
            errors.append(f"{name}: interface is missing")
            continue
        linkinfo = entry.get("linkinfo")
        info_kind = linkinfo.get("info_kind") if isinstance(linkinfo, dict) else None
        if entry.get("link_type") != "can" and info_kind != "can":
            errors.append(f"{name}: interface is not CAN")
        flags = entry.get("flags")
        if not isinstance(flags, list) or "UP" not in {str(flag).upper() for flag in flags}:
            errors.append(f"{name}: interface is not UP")
        if "LISTEN-ONLY" not in _ctrlmodes(entry):
            errors.append(f"{name}: kernel LISTEN-ONLY is not enabled")
        else:
            listen_only.append(name)

    return SocketCanRxPreflightReport(
        expected_interfaces=expected,
        observed_interfaces=tuple(sorted(entries)),
        listen_only_interfaces=tuple(listen_only),
        errors=tuple(errors),
    )
