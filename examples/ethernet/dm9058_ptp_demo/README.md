| Supported Targets | ESP32-S3 + DM9058 (SPI Ethernet) |
| ----------------- | -------------------------------- |

# PTP Time Synchronization with DM9058

This standalone project follows the same idea as ESP-IDF [`examples/ethernet/ptp`](https://github.com/espressif/esp-idf/blob/master/examples/ethernet/ptp/README.md) but uses an external **Davicom DM9058** PHY/MAC over **SPI**. General ESP-IDF usage: [Getting Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html).

## Overview

The project demonstrates **IEEE 1588-2008 (PTPv2)** time synchronization over Ethernet on ESP-IDF. It brings up **SPI-attached DM9058**, runs a PTP daemon ported from [NuttX ptpd](https://github.com/apache/nuttx-apps/tree/master/netutils/ptpd), and toggles a **GPIO** after the PTP clock is usable so you can observe timing with a scope or logic analyzer.

Unlike the on-chip **EMAC** reference example, timestamps come from the **DM9058 hardware PTP** block:

- **L2 (IEEE 802.3, EtherType 0x88F7)** or **UDP/IPv4**, selected in `menuconfig` and matched in `esp_eth_clock_init()` (`esp_eth_clock_cfg_t.transport`).
- **RX**: timestamped frames are delivered through **L2 TAP** together with the `esp_eth` DM9058 driver.
- **TX**: two-step or one-step event messages are coordinated with `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` and `ETH_MAC_DM9058_CMD_S_PTP_TWO_STEP_SYNC`; see `sdkconfig.defaults` and ESP-IDF `components/esp_eth` (DM9058 PTP sources).

## Supported DM9058 PTP Features

| Feature | Notes |
|--------|--------|
| PTP hardware clock | `esp_eth_ioctl`: e.g. `G/S_PTP_TIME`, `ADJ_PTP_FREQ`, `G_TX_TIMESTAMP` |
| L2 / UDP transport | `ETH_MAC_DM9058_CMD_S_PTP_TRANSPORT` + `esp_eth_clock_cfg_t.transport` |
| Two-step / one-step SYNC | Kconfig `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC` and MAC `DM9058_PTP_ONESTEP` / `prepare_tx` |
| Demo pulse GPIO | `CONFIG_EXAMPLE_PTP_PULSE_WIDTH_NS`, `CONFIG_EXAMPLE_PTP_PULSE_GPIO` |
| Target-time callback | On DM9058 this is **software `esp_timer`** (not EMAC hardware TS compare); see **GPIO measurement** below |

## Demonstration Setups

### A. Two DM9058 boards (recommended baseline)

- Flash one board as **PTP Server (Master)** and the other as **PTP Client (Slave)**.
- Connect with a short Ethernet cable **direct** or through a switch that forwards **PTP L2 multicast** (01-1b-19-00-00-00) / UDP ports **319/320** as required by your profile.
- Optional: scope or logic analyzer on the pulse GPIO of each board (see limitations below).

### B. One DM9058 board + PC (either side can be Grandmaster)

You can pair **one ESP32-S3 + DM9058** with a **Linux PC** running **linuxptp** (`ptp4l`) for interoperability tests.

| Role | Device | Notes |
|------|--------|--------|
| Grandmaster | PC | Run `ptp4l` on the wired interface; ensure multicast egress (L2 or UDP matches the ESP build). |
| Slave | ESP + DM9058 | Build as **PTP Client**; use **L2** or **UDP/IPv4** to match `ptp4l` (`-2` for L2, default UDP ports for UDP). |
| Grandmaster | ESP + DM9058 | Build as **PTP Server**; PC runs `ptp4l` in **slave** mode (`-s` / client profile per your distro docs). |
| Slave | PC | Same, with roles swapped. |

**PC hints (typical `ptp4l`):**

- **L2 PTP (802.3)**: e.g. `sudo ptp4l -i <iface> -m -2` (aligns with `EXAMPLE_PTP_TRANSPORT_L2` on ESP).
- **UDP/IPv4**: configure ESP for IPv4 PTP transport and ensure the PC has an address on the same subnet; `ptp4l` uses UDP **319/320** by default.
- **Two-step vs one-step on PC**: default **two-step** SYNC is the most portable. **`--twoStepFlag=0` (one-step)** requires **NIC hardware one-step TX** support; if unsupported, the PC may emit **no** or **broken** PTP—verify with `sudo tcpdump -i <iface> -nn ether proto 0x88f7` (L2) or UDP/319 and `ethtool -T <iface>` on the PC. Lack of packets is usually a **PC/driver** issue, not the ESP slave.

## How to Use

### Hardware

- One or two **ESP32-S3** (or the SoC selected in `sdkconfig`) boards with **DM9058**, wired per this project’s `ethernet_init` / Kconfig (SPI, CS, INT, RST, PHY address, etc.).
- For PC tests: **Ethernet cable** between PC NIC and DM9058 PHY (or via a lab switch with multicast/PTP-friendly settings).

### Configuration

```bash
idf.py menuconfig
```

Typical options:

- **Example Configuration**: pulse GPIO, pulse width (ns), PTP transport (L2 vs IPv4).
- **PTP Daemon**: Server/Client, `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC`, Delay Request options, etc.
- **SPI Ethernet DM9058**: pins and SPI host must match your PCB.

Defaults are summarized in [`sdkconfig.defaults`](sdkconfig.defaults).

### Build, Flash, Monitor

```bash
idf.py set-target esp32s3
idf.py -p PORT build flash monitor
```

Replace `PORT` with your serial device. Exit monitor: `Ctrl-]`.

## Example Log (Slave, DM9058 cable to DM9058 Master)

The log below is from a **Slave** board with **another DM9058 board as Master**, **back-to-back** Ethernet. In the **PTP software observability domain**, `Local time` vs `remote time` differ by **sub-microsecond**; **`offset_ns` is on the order of tens of nanoseconds** and **`path_delay` hundreds of ns**, which shows **nanosecond-class clock tracking** for the DM9058 + ptpd loop (similar stabilized `offset_ns` as in the internal EMAC PTP example).

```
I (298551) ptpd: [PTP RX][L2] Sync seq=310 hw_ts=1115.552018865
I (298551) ptpd: Got sync packet, seq 310

I (298551) ptpd: Local time: 1115.552018865, remote time 1115.552018480

W (298561) ptpd: offset_ns       -26, adj   -798, drift_acc    -793, path_delay   359 ns

I (299561) ptpd: [PTP RX][L2] Sync seq=311 hw_ts=1116.562159547
I (299561) ptpd: Got sync packet, seq 311

I (299561) ptpd: Local time: 1116.562159547, remote time 1116.562159200

W (299571) ptpd: offset_ns       +12, adj   -791, drift_acc    -793, path_delay   359 ns
```

Interpretation:

- **`Local time` / `remote time` ~350–400 ns apart**: difference between message time fields and the local **RX hardware timestamp** inside the stack (PTP **time domain**).
- **`offset_ns` ~±26 ns**: loop estimate of clock error—direct evidence of **ns-class synchronization** in software.

## GPIO Pulse vs. Nanosecond Sync (important)

**Nanosecond-level figures in the log refer to the PTP logical clock and timestamp alignment**, not necessarily **nanosecond alignment of GPIO edges** on a logic analyzer.

On **DM9058 (SPI)** there is **no** on-chip EMAC-style “PTP target time reached → hardware IRQ”. This project implements `esp_eth_clock_set_target_time` / `esp_eth_clock_register_target_cb` in `components/esp_eth_time` using **`esp_timer` (task context)** plus **SPI reads of DM9058 PTP time**. That adds:

- **Timer task scheduling jitter** (often **microseconds to tens of µs** under load);
- **SPI / mutex latency** when reading PTP time;
- **Independent pulse phasing** on Master and Slave—GPIO toggles are **not** guaranteed to occur at the **same absolute PTP instant** on two boards.

Therefore: **seeing several µs between GPIO edges on two boards is expected** and **does not contradict** **ns-class `offset_ns`** in the UART log. For **GPIO edges locked to PTP with ns precision**, you need **hardware PTP compare / PPS output** (or external circuitry), not this demo’s software-generated pulse alone.

## Differences from `examples/ethernet/ptp` (on-chip EMAC)

| Item | Official PTP example (e.g. ESP32-P4 README) | This project (DM9058) |
|------|-----------------------------------------------|------------------------|
| Ethernet | On-chip EMAC | SPI **DM9058** |
| Timestamps | EMAC HW + L2 TAP | **DM9058 PTP** + L2 TAP |
| Pulse scheduling | Hardware TS target interrupt | **Software `esp_timer`** |

Upstream reference README: [`examples/ethernet/ptp/README.md`](https://github.com/espressif/esp-idf/blob/master/examples/ethernet/ptp/README.md).

## Troubleshooting

- **Link**: ensure DM9058 link is **up** before expecting PTP traffic.
- **Master/Slave roles**: one device Server, one Client; if using a switch, confirm **L2 multicast** or **UDP 319/320** is not filtered unexpectedly.
- **PC one-step SYNC**: as above, `ptp4l --twoStepFlag=0` needs **NIC one-step TX**; use `tcpdump` / `ethtool -T` on the PC if the ESP sees **no** PTP packets.

For ESP-IDF–related issues, use [Espressif GitHub Issues](https://github.com/espressif/esp-idf/issues).
