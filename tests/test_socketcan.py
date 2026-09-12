import unittest

from open_sprite_runtime.socketcan import audit_socketcan_rx_snapshot


def entry(name: str, *, up: bool = True, listen_only: bool = True) -> dict:
    modes = ["FD"]
    if listen_only:
        modes.append("LISTEN-ONLY")
    return {
        "ifname": name,
        "flags": ["NOARP", "UP"] if up else ["NOARP"],
        "link_type": "can",
        "linkinfo": {"info_kind": "can", "info_data": {"ctrlmode": modes}},
    }


class SocketCanRxPreflightTests(unittest.TestCase):
    def test_four_up_listen_only_interfaces_pass_without_tx(self) -> None:
        report = audit_socketcan_rx_snapshot(
            [entry(f"can{index}") for index in range(4)],
            [f"can{index}" for index in range(4)],
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.to_dict()["hardware_tx_attempts"], 0)

    def test_missing_down_or_active_interface_fails_closed(self) -> None:
        report = audit_socketcan_rx_snapshot(
            [entry("can0"), entry("can1", up=False), entry("can2", listen_only=False)],
            ["can0", "can1", "can2", "can3"],
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("can1: interface is not UP" in error for error in report.errors))
        self.assertTrue(any("can2: kernel LISTEN-ONLY" in error for error in report.errors))
        self.assertTrue(any("can3: interface is missing" in error for error in report.errors))

    def test_non_can_interface_fails_closed(self) -> None:
        snapshot = [entry(f"can{index}") for index in range(4)]
        snapshot[2]["link_type"] = "ether"
        snapshot[2]["linkinfo"]["info_kind"] = "ether"
        report = audit_socketcan_rx_snapshot(snapshot, [f"can{index}" for index in range(4)])
        self.assertFalse(report.passed)
        self.assertTrue(any("can2: interface is not CAN" in error for error in report.errors))


if __name__ == "__main__":
    unittest.main()
