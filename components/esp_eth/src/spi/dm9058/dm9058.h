/*
 * SPDX-FileCopyrightText: 2019-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers in DM9058
 *
 */
#define DM9058_NCR (0x00)     // Network Control Register
#define DM9058_NSR (0x01)     // Network Status Register
#define DM9058_TCR (0x02)     // Tx Control Register
#define DM9058_TSR1 (0x03)    // Tx Status Register I
#define DM9058_TSR2 (0x04)    // Tx Status Register II
#define DM9058_RCR (0x05)     // Rx Control Register
#define DM9058_RSR (0x06)     // Rx Status Register
#define DM9058_ROCR (0x07)    // Receive Overflow Counter Register
#define DM9058_BPTR (0x08)    // Back Pressure Threshold Register
#define DM9058_FCTR (0x09)    // Flow Control Threshold Register
#define DM9058_FCR (0x0A)     // Rx/Tx Flow Control Register
#define DM9058_EPCR (0x0B)    // EEPROM & PHY Control Register
#define DM9058_EPAR (0x0C)    // EEPROM & PHY Address Register
#define DM9058_EPDRL (0x0D)   // EEPROM & PHY Data Register Low
#define DM9058_EPDRH (0x0E)   // EEPROM & PHY Data Register High
#define DM9058_WCR (0x0F)     // Wake Up Control Register
#define DM9058_PAR (0x10)     // Physical Address Register
#define DM9058_MAR (0x16)     // Multicast Address Hash Table Register
#define DM9058_GPCR (0x1E)    // General Purpose Control Register
#define DM9058_GPR (0x1F)     // General Purpose Register
#define DM9058_TRPAL (0x22)   // Tx Memory Read Pointer Address Low Byte
#define DM9058_TRPAH (0x23)   // Tx Memory Read Pointer Address High Byte
#define DM9058_RWPAL (0x24)   // Rx Memory Read Pointer Address Low Byte
#define DM9058_RWPAH (0x25)   // Rx Memory Read Pointer Address High Byte
#define DM9058_VIDL (0x28)    // Vendor ID Low Byte
#define DM9058_VIDH (0x29)    // Vendor ID High Byte
#define DM9058_PIDL (0x2A)    // Product ID Low Byte
#define DM9058_PIDH (0x2B)    // Product ID High Byte
#define DM9058_CHIPR (0x2C)   // CHIP Revision
#define DM9058_TCR2 (0x2D)    // Transmit Control Register 2
#define DM9058_OTCR (0x2E)    // Operation Test Control Register
#define DM9058_SMCR (0x2F)    // Special Mode Control Register
#define DM9058_ATCR (0x30)    // Auto-Transmit Control Register
#define DM9058_CSCR (0x31)    // Transmit Check Sum Control Register
#define DM9058_RCSSR (0x32)   // Receive Check Sum Control Status Register
#define DM9058_PBCR (0x38)    // SPI Bus Control Register
#define DM9058_INTR (0x39)    // INT Pin Control Register
#define DM9058_TXFSSR (0x3B)  // TX FIFO Status Register
#define DM9058_PPCR (0x3D)    // Pause Packet Control Register
#define DM9058_EEE_IN (0x3E)  // IEEE 802.3az Enter Counter Register
#define DM9058_EEE_OUT (0x3F) // IEEE 802.3az Leave Counter Register
#define DM9058_ALNCR (0x4A)   // SPI Byte Align Error Counter Register
#define DM9058_RLENCR (0x52)  // Rx Packet Length Control Register
#define DM9058_BCASTCR (0x53) // RX Broadcast Control Register
#define DM9058_IPCOCR (0x54)  // IP/ICMP Checksum Offload Control Register
#define DM9058_MPCR (0x55)    // Memory Pointer Control Register
#define DM9058_LMCR (0x57)    // LED Mode Control Register
#define DM9058_MBNDRY (0x5E)  // Memory Boundary Register
#define DM9058_MRCMDX (0x70)  // Memory Data Pre-Fetch Read Command Without Address Increment Register
#define DM9058_MRCMDX1 (0x71) // Memory Read Command Without Pre-Fetch and Without Address Increment Register
#define DM9058_MRCMD (0x72)   // Memory Data Read Command With Address Increment Register
#define DM9058_SDR_DLY (0x73) // SPI Data Read Delay Counter Register
#define DM9058_MRRL (0x74)    // Memory Data Read Address Register Low Byte
#define DM9058_MRRH (0x75)    // Memory Data Read Address Register High Byte
#define DM9058_MWCMDX (0x76)  // Memory Data Write Command Without Address Increment Register
#define DM9058_MWCMD (0x78)   // Memory Data Write Command With Address Increment Register
#define DM9058_MWRL (0x7A)    // Memory Data Write Address Register Low Byte
#define DM9058_MWRH (0x7B)    // Memory Data Write Address Register High Byte
#define DM9058_TXPLL (0x7C)   // TX Packet Length Low Byte Register
#define DM9058_TXPLH (0x7D)   // TX Packet Length High Byte Register
#define DM9058_ISR (0x7E)     // Interrupt Status Register
#define DM9058_IMR (0x7F)     // Interrupt Mask Register

/**
 * @brief PTP (IEEE 1588) Registers in DM9058
 *
 */
#define DM9058_PTP_CR (0x60)      // PTP Control Register
#define DM9058_PTP_ENR (0x61)     // PTP Enable/Index Register
#define DM9058_PTP_TXCR (0x62)    // PTP TX Timestamp Control Register
#define DM9058_PTP_ONESTEP (0x63) // PTP One-Step TX Control Register
#define DM9058_PTP_RXCR (0x64)    // PTP RX Control Register
#define DM9058_PTP_TSOFF (0x65)   // PTP Timestamp Offset Register
#define DM9058_PTP_CSOFF (0x66)   // PTP Checksum Offset Register
#define DM9058_PTP_DATA (0x68)    // PTP Data Register (8 bytes: ns[0-3], sec[4-7])
#define DM9058_PTP_RATE (0x69)    // PTP Rate Read Enable Register

/* PTP Control Register (0x60) bits */
#define PTP_CR_RESTART (1 << 0)   // PTP Restart (write 1 then 0)

/* PTP Enable/Index Register (0x61) bits */
#define PTP_ENR_ENABLE (1 << 0)   // PTP Enable
#define PTP_ENR_SETTIME (1 << 3)  // Write PTP Clock Time
#define PTP_ENR_GETTIME (1 << 2)  // Read PTP Clock Time
#define PTP_ENR_ADDOFF (1 << 4)   // Add Time Offset
#define PTP_ENR_ADJSLOWER (1 << 5)// Adjust Slower
#define PTP_ENR_ADJFASTER (1 << 5)// Adjust Faster (without bit6)
#define PTP_ENR_RSTIDX (1 << 7)   // Reset Data Index

/* PTP TX Timestamp Control Register (0x62) bits */
#define PTP_TXCR_READTS (1 << 0)  // Read TX Timestamp

/* PTP RX Control Register (0x64) bits */
#define PTP_RXCR_ENABLE (1 << 4)  // Enable RX Timestamp
#define PTP_RXCR_MCAST (1 << 1)   // Multicast Mode

/* PTP adjustment constants */
#define PTP_ADJ_FREQ_BASE_ADDEND     171.7987f
#define PTP_ADJ_FREQ_BASE_ADDEND_Q16 11259106
#define PTP_ADJ_MAX                  0xEFFFFFFF
#define PTP_ADJUST_SLOWER_CTRL       0x60
#define PTP_ADJUST_FASTER_CTRL       0x20

/**
 * @brief status and flag of DM9058 specific registers
 *
 */
#define DM9058_SPI_RD (0) // Burst Read Command
#define DM9058_SPI_WR (1) // Burst Write Command

#define NCR_EXT_PHY (1 << 7) // External PHY
#define NCR_WAKEEN (1 << 6)  // Enable Wakeup Function
#define NCR_FCOL (1 << 4)    // Force Collision Mode
#define NCR_FDX (1 << 3)     // Duplex Mode of the Internal PHY
#define NCR_LBK (3 << 1)     // Loopback Mode
#define NCR_RST (1 << 0)     // Software Reset and Auto-Clear after 10us

#define NSR_SPEED (1 << 7)  // Speed of Internal PHY
#define NSR_LINKST (1 << 6) // Link Status of Internal PHY
#define NSR_WAKEST (1 << 5) // Wakeup Event Status
#define NSR_TX2END (1 << 3) // TX Packet Index II Complete Status
#define NSR_TX1END (1 << 2) // TX Packet Index I Complete Status
#define NSR_RXOV (1 << 1)   // RX Memory Overflow Status
#define NSR_RXRDY (1 << 0)  // RX Packet Ready

#define TCR_RSV_BIT7 (1 << 7) // Reserved
#define TCR_TJDIS (1 << 6)    // Transmit Jabber Timer Disable
#define TCR_EXCECM (1 << 5)   // Excessive Collision Mode
#define TCR_PAD_DIS2 (1 << 4) // Disable Padding for Packet Index II
#define TCR_CRC_DIS2 (1 << 3) // Disable CRC for Packet Index II
#define TCR_PAD_DIS1 (1 << 2) // Disable Padding for Packet Index I
#define TCR_CRC_DIS1 (1 << 1) // Disable CRC for Packet Index I
#define TCR_TXREQ (1 << 0)    // TX Request. Auto-Clear after Sending Completely

#define RCR_WTDIS (1 << 6)     // Watchdog Timer Disable
#define RCR_DIS_LONG (1 << 5)  // Discard Long Packet
#define RCR_DIS_CRC (1 << 4)   // Discard CRC Error Packet
#define RCR_ALL_MCAST (1 << 3) // Receive All Multicast
#define RCR_RUNT (1 << 2)      // Receive Runt Packet
#define RCR_PRMSC (1 << 1)     // Promiscuous Mode
#define RCR_RXEN (1 << 0)      // RX Enable

#define RSR_RF (1 << 7)   // Runt Frame
#define RSR_MF (1 << 6)   // Multicast Frame
#define RSR_LCS (1 << 5)  // Late Collision Seen
#define RSR_RWTO (1 << 4) // Receive Watchdog Time-Out
#define RSR_PLE (1 << 3)  // Physical Layer Error
#define RSR_AE (1 << 2)   // Alignment Error
#define RSR_CE (1 << 1)   // CRC Error
#define RSR_FOE (1 << 0)  // RX Memory Overflow Error

#define FCR_TXPEN (1 << 5) // TX Pause Packet Enable
#define FCR_BKPA (1 << 4)  // Back Pressure Active
#define FCR_BKPM (1 << 3)  // Back Pressure Mode
#define FCR_FLCE (1 << 0)  // Flow Control Enable
#define FCR_FLOW_ENABLE (0x39) // Enable Flow Control

#define EPCR_REEP (1 << 5)  // Reload EEPROM
#define EPCR_WEP (1 << 4)   // Write EEPROM Enable
#define EPCR_EPOS (1 << 3)  // EEPROM or PHY Operation Select
#define EPCR_ERPRR (1 << 2) // EEPROM Read or PHY Register Read Command
#define EPCR_ERPRW (1 << 1) // EEPROM Write or PHY Register Write Command
#define EPCR_ERRE (1 << 0)  // EEPROM Access Status or PHY Access Status

#define TCR2_RLCP (1 << 6) // Retry Late Collision Packet
#define TCR2_DTU (1 << 4)  // Dual Transmit Unit

#define ATCR_AUTO_TX (1 << 7) // Auto-Transmit Control

#define CSCR_UDPCSE (1 << 2) // UDP CheckSum Generation
#define CSCR_TCPCSE (1 << 1) // TCP CheckSum Generation
#define CSCR_IPCSE (1 << 0)  // IPv4 CheckSum Generation

#define RCSSR_UDPS (1 << 7)  // UDP Checksum Status
#define RCSSR_TCPS (1 << 6)  // TCP Checksum Status
#define RCSSR_IPS (1 << 5)   // IP Checksum Status
#define RCSSR_UDPP (1 << 4)  // UDP Packet
#define RCSSR_TCPP (1 << 3)  // TCP Packet
#define RCSSR_IPP (1 << 2)   // IP Packet
#define RCSSR_RCSEN (1 << 1) // Receive Checksum Checking Enable
#define RCSSR_DCSE (1 << 0)  // Discard Checksum Error Packet

#define INTR_ACTIVE_LOW (1 << 0) // INT Pin Active Low

#define MPCR_RST_TX (1 << 1) // Reset TX Memory Pointer
#define MPCR_RST_RX (1 << 0) // Reset RX Memory Pointer

#define MBNDRY_BYTE (1 << 7) // Memory Boundary Byte Mode
#define MBNDRY_WORD (0)      // Memory Boundary Word Mode

#define IPCOCR_CLKOUT (1 << 7) // Clock Output Enable

#define LMCR_NEWMOD (1 << 7)  // New LED Mode
#define LMCR_POL (1 << 2)     // Reverse Polarity of LED Type
#define LMCR_TYPED0 (1 << 0)  // LED Type D0
#define LMCR_TYPED1 (1 << 1)  // LED Type D1
#define LMCR_MODE1 (LMCR_NEWMOD | LMCR_TYPED0) // LED Mode 1

#define ISR_LNKCHGS (1 << 5) // Link Status Change
#define ISR_ROO (1 << 3)     // Receive Overflow Counter Overflow
#define ISR_ROS (1 << 2)     // Receive Overflow
#define ISR_PT (1 << 1)      // Packet Transmitted
#define ISR_PR (1 << 0)      // Packet Received
#define ISR_CLR_STATUS (ISR_LNKCHGS | ISR_ROO | ISR_ROS | ISR_PT | ISR_PR)

#define IMR_PAR (1 << 7)     // Pointer Auto-Return Mode
#define IMR_LNKCHGI (1 << 5) // Enable Link Status Change Interrupt
#define IMR_ROOI (1 << 3)    // Enable Receive Overflow Counter Overflow Interrupt
#define IMR_ROI (1 << 2)     // Enable Receive Overflow Interrupt
#define IMR_PTI (1 << 1)     // Enable Packet Transmitted Interrupt
#define IMR_PRI (1 << 0)     // Enable Packet Received Interrupt
#define IMR_ALL (IMR_PAR | IMR_LNKCHGI | IMR_ROOI | IMR_ROI | IMR_PTI | IMR_PRI)

#define PPCR_PAUSE_COUNT (0x0F) // Pause Packet Control Count

#ifdef __cplusplus
}
#endif
