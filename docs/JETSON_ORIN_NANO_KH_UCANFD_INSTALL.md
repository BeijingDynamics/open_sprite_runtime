# Jetson Orin Nano KH-UCANFD installation

Installation date: 2026-09-14

## Target

- Host: `tony-jetson` (`192.168.0.253` at installation time)
- Platform: NVIDIA Jetson Orin Nano, `aarch64`
- OS: Ubuntu 22.04.5 LTS
- Kernel: `5.15.199-tegra`
- Kernel headers: `/usr/src/linux-headers-5.15.199-tegra-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source`
- Kernel compiler: GCC 11.3; installed compiler: Ubuntu GCC 11.4

## Installed software

The driver was installed from the official KunHong release archive using
`./build.sh -rules` as described in the official Linux SDK installation guide.

- SDK: `KH-UCANFD_Linux_SDK-1.4.2`
- SDK archive SHA-256: `0859afc67dc63f232d64a8815a4c2a7bcf7129298c48748a70174647a4a6508c`
- Installed module: `/lib/modules/5.15.199-tegra/misc/kcan.ko`
- Module SHA-256: `d2d68c051e5d9ee7cda59cdce38362681d2f15bb1049eabedc6dcb959bdb8576`
- Module version: `Release_1.4.2_20260721_n`
- Module vermagic: `5.15.199-tegra SMP preempt mod_unload modversions aarch64`
- Udev rule: `/etc/udev/rules.d/100-kcan.rules`
- Installed tools: `lskcan`, `kcan_monitor`, `kcanfd_test`, `kcanfdtest`,
  `kcan-settings`, `kcan_fw_tool`, and `kcan_fw_upgrade`

The udev rule names KunHong netdev interfaces `kcan%n`. This keeps them distinct
from the Jetson's native `can0`. Runtime logical channel 0 through 3 must be
bound to physical `CANFD1` through `CANFD4` only after actual enumeration is
recorded.

## Verified device enumeration

The module compiled, installed, and loaded successfully. At verification time,
`lsmod` showed `kcan`, and `/sys/class/kcan/version` reported `1.4.2`.

After the dongle was connected, USB ID `395e:0020` enumerated as
`KCAN KCAN-USB X4` and bound to driver `kcan`. The driver reported adapter 0
with four controller numbers:

| Physical channel | Controller | Character device | SocketCAN interface |
|---|---:|---|---|
| CANFD1 | 0 | `kcanusbfd32` | `kcan1` |
| CANFD2 | 1 | `kcanusbfd33` | `kcan2` |
| CANFD3 | 2 | `kcanusbfd34` | `kcan3` |
| CANFD4 | 3 | `kcanusbfd35` | `kcan4` |

All four channels were `DOWN`/`CLOSED`, with zero RX frames, zero TX frames,
and zero error counters. The device reported firmware `3.2.0` and bootloader
`1.1.0`; no firmware update was attempted.

The following gates are still incomplete:

1. Connect the four physical CAN FD buses with robot power disabled.
2. Run receive-only/listen-only validation before configuring bitrate or any
   transmit-capable motor test.

Do not infer channel order from interface enumeration, and do not transmit to
motors until the runtime's receive-only audit and all hardware safety gates pass.
