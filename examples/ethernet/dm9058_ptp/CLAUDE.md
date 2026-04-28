# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP-IDF example demonstrating IEEE 1588 PTPv2 time synchronization over Ethernet at Layer 2 (L2 TAP interface, EtherType `0x88F7`, per Annex F of IEEE 1588-2008). Hardware timestamps come from the Ethernet MAC; the daemon is a NuttX PTPD port adapted for ESP-IDF. Two devices run the same firmware and align a GPIO pulse so the sync precision can be measured on an oscilloscope.

The upstream example targets the ESP32-P4 internal EMAC; **this fork adds DM9058 SPI Ethernet PHY support and targets ESP32-S3** (see `sdkconfig.defaults`). The DM9058 PTP support (`ESP_ETH_PTP_DM9058_TRANSPORT_*`, `NETUTILS_PTPD_TWOSTEP_SYNC`) lives upstream in `$IDF_PATH/components/esp_eth/src/spi/dm9058/` — that is a modified ESP-IDF tree, not the stock release.

## Build / flash / monitor

ESP-IDF v5.5.2 is installed at `C:\esp\v5.5.2\esp-idf`. Each new shell must first source the export script:

```powershell
c:\esp\v5.5.2\esp-idf\export.ps1      # PowerShell
# or
& D:\prg\esp-idf\esp-idf-davicom\export.ps1   # alternate install used during development
```

Then from this directory:

```bash
idf.py set-target esp32s3      # target is ESP32-S3, not P4 as README claims
idf.py menuconfig              # configure PTP role, DM9058 two-step mode, etc.
idf.py build
idf.py -p COMx flash monitor   # exit monitor with Ctrl-]
```

`set-target` wipes `sdkconfig` — re-copy `sdkconfig.defaults` → `sdkconfig` after retargeting if you want the DM9058 defaults back.

## Architecture

Three layers, bottom-up:

1. **DM9058 PTP driver** (`$IDF_PATH/components/esp_eth/src/spi/dm9058/`, outside this example). Exposes `ETH_MAC_ESP_CMD_*` ioctls (`PTP_ENABLE`, `G/S_PTP_TIME`, `ADJ_PTP_FREQ`, `G_PTP_RX_TIME`, `S_TARGET_TIME`, `S_TARGET_CB`) and the `esp_eth_ptp_dm9058_transport_t` enum (`UDP_IPV4`, `UDP_IPV6`, `IEEE_802_3`, `IEEE_802_1AS`). Two-step timestamp mode is controlled by `CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC`.

2. **`components/esp_eth_time/`** — thin POSIX-style wrapper over those ioctls. Exposes `CLOCK_PTP_SYSTEM` (id 19) and `clock_gettime/settime/adjtime`-shaped functions plus target-time callback registration. `s_eth_hndl` is a file-static set in `esp_eth_clock_init()`, so this component assumes **one** PTP-capable ethernet handle per process.

3. **`components/ptpd/`** — NuttX `apps/netutils/ptpd` ported to ESP-IDF, gated by `#define ESP_PTP 1` in `ptpd.c`. Instead of UDP sockets on port 319/320 (the NuttX path), the ESP port opens an L2 TAP fd (`/dev/net/tap`), filters on EtherType `0x88F7`, and reads/writes raw Ethernet frames. Public API is just `ptpd_start(iface)` / `ptpd_status(pid, &status)` / `ptpd_stop(pid)`. Kconfig selects master (`NETUTILS_PTPD_SERVER`) / slave (`NETUTILS_PTPD_CLIENT`) / both; `NETUTILS_PTPD_TWOSTEP_SYNC` defaults to follow the DM9058 hardware two-step flag so protocol behavior matches the timestamp mode — keep these in sync.

4. **`main/ptp_main.c`** — glues it together: `example_eth_init` → `esp_netif_init` → `esp_vfs_l2tap_intf_register` → create netifs as `ETH_0..N` → `esp_eth_start` → wait for `ETHERNET_EVENT_CONNECTED` → `esp_eth_clock_init` (currently hard-coded to `ESP_ETH_PTP_DM9058_TRANSPORT_IEEE_802_3`) → register pulse ISR callback → `ptpd_start("ETH_0")`. Main loop polls `ptpd_status()`; after 3 consecutive `clock_source_valid` reads (slave) or on first pass (master) it arms the next GPIO target time via `esp_eth_clock_set_target_time`, and the MAC's target-exceed interrupt fires `ts_callback` (IRAM-resident) which toggles `CONFIG_EXAMPLE_PTP_PULSE_GPIO` and re-arms.

The slave's `clock_source_valid` debounce is not in the Nuttx original — it is specific to this example and prevents pulse restarts on a single lost announce packet.

## Conventions worth knowing

- **L2 TAP interface key = `ETH_<index>`** (matches `if_key` set in `init_ethernet_and_netif`). `ptpd_start` takes this key, not a POSIX `eth0`-style name.
- **One ethernet handle** — `esp_eth_time` stores `s_eth_hndl` statically. Multi-port configs need refactoring before the clock API can be used on ports other than `eth_handles[0]`.
- **`CLOCK_PTP_SYSTEM` is a private clockid (19)**, not portable. All `esp_eth_clock_*` calls switch on it; passing anything else returns `EINVAL`.
- **Transport hard-coded in `ptp_main.c`** — change `clock_cfg.transport` there, not only in Kconfig, when experimenting with IEEE 802.3 vs 802.1AS vs UDP transports. The branch name `L2TAP_two_step_e2e_v001` reflects the current focus: L2 (802.3) transport with two-step E2E delay mechanism.
- **Upstream drift** — `ptpd.c` preserves the original NuttX sources behind `#ifndef ESP_PTP` blocks; keep those intact when merging upstream ptpd fixes so the diff stays reviewable.

## Files outside this tree that the example depends on

- `$IDF_PATH/examples/ethernet/basic/components/ethernet_init` — `example_eth_init` (board pin / PHY selection).
- `$IDF_PATH/components/esp_eth/src/spi/dm9058/` — DM9058 MAC + PTP driver (modified in this IDF tree).
- `$IDF_PATH/DM9058_PTP_整合指南.md` — integration notes for the DM9058 PTP additions.
