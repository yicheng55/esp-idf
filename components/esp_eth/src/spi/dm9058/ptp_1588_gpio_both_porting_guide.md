# DM9051 PTP 1588 GPIO 雙通道觸發功能移植指南

> 來源檔案：`dm9051_diag_f437/developer/devel2/diag/src/ptp_functions.c`  
> 函式名稱：`ptp_1588_gpio_both()`  
> 文件日期：2026-05-06

---

## 目錄

1. [功能概述](#功能概述)
2. [依賴的標頭檔與原始檔](#依賴的標頭檔與原始檔)
3. [資料型態定義](#資料型態定義)
4. [常數與巨集定義](#常數與巨集定義)
5. [暫存器對應表](#暫存器對應表)
6. [核心函式說明](#核心函式說明)
7. [gpio_trigger_params_t 參數詳解](#gpio_trigger_params_t-參數詳解)
8. [脈衝寬度計算公式](#脈衝寬度計算公式)
9. [暫存器操作序列](#暫存器操作序列)
10. [ptp_1588_gpio_both 完整流程](#ptp_1588_gpio_both-完整流程)
11. [移植注意事項](#移植注意事項)

---

## 功能概述

`ptp_1588_gpio_both()` 函式同時設定並啟動 **GP1 (GPIO0)** 及 **GP2 (GPIO1)** 兩個 PTP 1588 GPIO 通道，皆工作在**輸出模式**的**單脈衝 (Single Pulse)** 觸發方式。

主流程如下：
1. 初始化 GP1 / GP2 各自的 `gpio_trigger_params_t` 參數結構。
2. 呼叫 `print_gpio_trigger_params03()` 顯示（並允許互動式修改）參數。
3. 呼叫 `test_1588_gpio_trigger04()` 將參數寫入 DM9051 暫存器並啟動觸發。
4. 進入主迴圈，持續呼叫 `wait_for_gpio_event_or_trigger()` 監控觸發完成狀態。
5. 收到 ESC 鍵時退出迴圈。

---

## 依賴的標頭檔與原始檔

| 檔案 | 用途 |
|------|------|
| `ptp_types.h` | `ptp_time_t`、`ptp_msg_type_t` 等基礎型態 |
| `dm9051a_ptp_1588.h` | 暫存器位址巨集、`ptp_clk_type_t`、`dm9051a_ptp_get_time()` 等宣告 |
| `dm9051a_ptp_1588.c` | `dm9051a_ptp_get_time()` / `dm9051a_ptp_set_time()` 實作 |
| `ptp_functions.c` | 本功能所有核心函式的實作 |

---

## 資料型態定義

### `ptp_time_t`（定義於 `ptp_types.h`）

```c
typedef struct {
    uint16_t reverse;       // 保留欄位（對齊用）
    uint32_t seconds;       // 秒數（32-bit）
    uint32_t nanoseconds;   // 奈秒數（32-bit）
} ptp_time_t;
```

### `gpio_trigger_params_t`（定義於 `ptp_functions.c` 第 941 行）

```c
typedef struct
{
    unsigned int GPIO_pin;       // GPIO 腳位選擇：0 = GP1, 1 = GP2
    unsigned int is_input_mode;  // 模式：0 = 輸出, 1 = 輸入
    unsigned int event_mode;     // 輸入模式：事件邊緣 0 = 上升沿, 1 = 下降沿
    unsigned int trigger_mode;   // 輸出模式：觸發類型
                                 //   0 = Edge (邊緣)
                                 //   1 = Toggle (翻轉)
                                 //   2 = Single Pulse (單脈衝)
                                 //   3 = Periodic Pulse (週期脈衝)
    unsigned int trig_por;       // 觸發極性：0 = Active Low, 1 = Active High
    unsigned int sec_offset;     // 觸發時間偏移（秒），相對於當前 PTP 時鐘加上此偏移

    struct {
        unsigned int asserted_width_unit;    // 斷言脈衝單位：0=120ns, 1=1.024µs, 2=1ms
        unsigned int asserted_width_high;    // 斷言脈衝高位寬度（6-bit，0–63）
        unsigned int asserted_width_low;     // 斷言脈衝低位寬度（8-bit，0–255）

        unsigned int deasserted_width_unit;  // 去斷言脈衝單位：0=120ns, 1=1.024µs, 2=1ms
        unsigned int deasserted_width_high;  // 去斷言脈衝高位寬度（6-bit，0–63）
        unsigned int deasserted_width_low;   // 去斷言脈衝低位寬度（8-bit，0–255）
    } pulse_params;

    ptp_time_t ts;               // 計算後的觸發目標時間（由函式內部填入）
} gpio_trigger_params_t;
```

### `ptp_clk_type_t`（定義於 `dm9051a_ptp_1588.h`）

```c
typedef enum {
    PTP_CLOCK_DISABLE = 0x02,
    PTP_CLOCK_ENABLE  = 0x01,
    PTP_WRITE_CLOCK   = 0x09,
    PTP_READ_CLOCK    = 0x04,
    PTP_WRITE_OFFSET  = 0x10,
    PTP_WRITE_RATE    = 0x20,
    PTP_SUB_RATE      = 0x60,
    RESET_IDX         = 0x80
} ptp_clk_type_t;
```

---

## 常數與巨集定義

以下常數定義於 `ptp_functions.c` 第 1284–1290 行：

```c
// GPIO 事件讀取參數（寫入 REG[0x62] 以觸發事件讀取）
#define GPIO0_EVENT_READ  0x10  // GPIO0 (GP1) 事件讀取啟動值
#define GPIO1_EVENT_READ  0x20  // GPIO1 (GP2) 事件讀取啟動值

// REG[0x60] 狀態位元映射（Write-1-to-Clear）
#define GPIO0_TRIGGER_COMPLETE_BIT  (1 << 5)  // bit5：GP1 觸發完成旗標
#define GPIO1_TRIGGER_COMPLETE_BIT  (1 << 7)  // bit7：GP2 觸發完成旗標
```

REG[0x60] 中斷狀態位元對照：

| 位元 | 值 | 意義 |
|------|----|------|
| bit5 | 0x20 | GP1 (GPIO0) 輸入事件已偵測 / 輸出觸發完成 |
| bit7 | 0x80 | GP2 (GPIO1) 輸入事件已偵測 / 輸出觸發完成 |

---

## 暫存器對應表

定義於 `dm9051a_ptp_1588.h`：

| 巨集名稱 | 位址 | 說明 |
|----------|------|------|
| `DM9051_REG_PSGR`  | `0x60` | PTP 系統一般暫存器（狀態/中斷旗標） |
| `DM9051_REG_PCCR`  | `0x61` | PTP 時鐘控制暫存器 |
| `DM9051_REG_PTPCR` | `0x62` | PTP 計時器控制暫存器（啟動 GPIO 事件/觸發） |
| `DM9051_REG_PSTR`  | `0x68` | PTP 系統時間暫存器（8 bytes，自動遞增索引） |
| `DM9051_REG_PGCR`  | `0x6A` | PTP GPIO 控制暫存器 |
| `DM9051_REG_PGTER` | `0x6B` | PTP GPIO 觸發設定暫存器 |
| —                  | `0x6C` | 斷言脈衝低位寬度（Asserted Low Width） |
| —                  | `0x6D` | 斷言脈衝高位寬度 + 單位（Asserted High Width + Unit） |
| —                  | `0x6E` | 去斷言脈衝低位寬度（Deasserted Low Width） |
| —                  | `0x6F` | 去斷言脈衝高位寬度 + 單位（Deasserted High Width + Unit） |

---

## 核心函式說明

### 1. `test_1588_gpio_trigger04(gpio_trigger_params_t *params)`

將 `gpio_trigger_params_t` 中的參數完整寫入 DM9051 暫存器，並啟動觸發計時。

**操作步驟（輸出模式）：**

```
REG[0x61] 確認 bit0 = 1（PTP 已啟用）
REG[0x60] = GPIO_pin ? 0x02 : 0x00         ← 選擇 GP2 或 GP1
REG[0x6A] = 0x06 | is_input_mode           ← GPIO 控制模式（0x06 = 觸發/中斷啟用）
REG[0x6B] = trigger_values[trigger_mode]
           | (trig_por ? 0x02 : 0x00)       ← 觸發模式 + 極性

// 若 trigger_mode == 2 或 3（脈衝模式）：
REG[0x6D] = unit_value | high_width         ← 斷言脈衝高位 + 單位
REG[0x6C] = low_width                       ← 斷言脈衝低位
REG[0x6F] = unit_value | high_width         ← 去斷言脈衝高位 + 單位
REG[0x6E] = low_width                       ← 去斷言脈衝低位

REG[0x61] = 0x84                            ← RESET_IDX + PTP_READ_CLOCK（讀當前時鐘）
// 讀取 REG[0x68] 取得當前 PTP 時間，加上 sec_offset 作為觸發目標時間
// 將觸發目標時間寫回 REG[0x68]（透過 test_1588_reg_68_wd01()）

REG[0x62] = 0x80 | (GPIO_pin ? 0x20 : 0x10) ← 啟動 GPIO 觸發，同時開啟中斷
```

**REG[0x6B] 觸發模式對照表：**

| `trigger_mode` | 查找表值 | 說明 |
|----------------|----------|------|
| 0 | `0x00` | Edge（邊緣） |
| 1 | `0x04` | Toggle（翻轉） |
| 2 | `0x08` | Single Pulse（單脈衝） |
| 3 | `0x0C` | Periodic Pulse（週期脈衝） |

加上極性位元：`REG[0x6B] |= (trig_por & 0x01) ? 0x02 : 0x00`

---

### 2. `wait_for_gpio_event_or_trigger(gpio_trigger_params_t *params, ptp_time_t *ts)`

輪詢等待觸發完成或輸入事件。

**輸出模式邏輯：**
- 若 `trigger_mode == 3`（週期模式）→ 直接返回 `FALSE`（週期模式不需等待）
- 讀取 `REG[0x60]`，檢查對應 GPIO 的觸發完成位元：
  - GP1：bit5（`GPIO0_TRIGGER_COMPLETE_BIT = 0x20`）
  - GP2：bit7（`GPIO1_TRIGGER_COMPLETE_BIT = 0x80`）
- 若該位元為 1 → 寫入同樣的值清除旗標（Write-1-to-Clear）→ 返回 `TRUE`

**輸入模式邏輯：**
- 呼叫 `dm9051_gpio_event_read_ptp_clock()` 讀取 REG[0x60] 的事件狀態位元：
  - GP1：bit5（0x20）；GP2：bit7（0x80）
- 偵測到事件 → 清除中斷旗標 → 設定 `REG[0x62]` 啟動時鐘讀取 → 讀 8 bytes from REG[0x68]

---

### 3. `dm9051a_ptp_get_time(PTP_READ_CLOCK, ptp_time_t *ts)`

從 DM9051 讀取當前 PTP 硬體時鐘。

```
REG[0x61] = 0x84     ← RESET_IDX(0x80) | PTP_READ_CLOCK(0x04)，重置 REG[0x68] 索引並觸發讀取
連續讀取 REG[0x68] 8 次（每次 1 byte，自動遞增索引）：
  buf[0..3] → nanoseconds（LSB first）
  buf[4..7] → seconds（LSB first）

ts->nanoseconds = buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24)
ts->seconds     = buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24)
```

---

### 4. `test_1588_reg_68_wd01(ptp_time_t ts)`

將指定時間戳寫入 DM9051 作為 GPIO 觸發目標時間。

```
REG[0x61] = 0x80     ← RESET_IDX，重置 REG[0x68] 索引
連續寫入 REG[0x68] 8 次（每次 1 byte，LSB first）：
  byte[0..3] = ts.nanoseconds（LSB first）
  byte[4..7] = ts.seconds（LSB first）
```

---

### 5. `print_gpio_trigger_params03(gpio_trigger_params_t *params)`

互動式顯示並編輯 `gpio_trigger_params_t` 參數的終端介面函式，在移植時可視需求保留或替換為自訂的參數設定流程。

---

## `gpio_trigger_params_t` 參數詳解

### `ptp_1588_gpio_both()` 中的預設值

| 欄位 | GP1 值 | GP2 值 | 說明 |
|------|--------|--------|------|
| `GPIO_pin` | `0` | `1` | 0=GP1, 1=GP2 |
| `is_input_mode` | `0` | `0` | 輸出模式 |
| `event_mode` | `0` | `0` | 上升沿（僅輸入模式有效） |
| `trigger_mode` | `2` | `2` | Single Pulse 單脈衝 |
| `trig_por` | `1` | `1` | Active High |
| `sec_offset` | `1` | `1` | 當前時間 + 1 秒後觸發 |
| `pulse_params.asserted_width_unit` | `2` | `2` | 1ms 單位 |
| `pulse_params.asserted_width_high` | `2` | `3` | 高位寬度 |
| `pulse_params.asserted_width_low` | `10` | `10` | 低位寬度 |
| `pulse_params.deasserted_width_unit` | `2` | `2` | 1ms 單位 |
| `pulse_params.deasserted_width_high` | `2` | `2` | 高位寬度 |
| `pulse_params.deasserted_width_low` | `20` | `20` | 低位寬度 |

---

## 脈衝寬度計算公式

### 脈衝單位對照（`*_width_unit`）

| 值 | 單位 | 暫存器高位元 |
|----|------|-------------|
| `0` | 120 ns | `0x00` |
| `1` | 1.024 µs | `0x40` |
| `2` | 1 ms | `0x80` |

計算公式（程式碼 `test_1588_gpio_trigger04` 中）：

```c
unit_value = (width_unit == 1) ? 0x40 :
             (width_unit == 2) ? 0x80 : 0x00;
```

### REG[0x6D] / REG[0x6F]（高位元暫存器）

```
REG[0x6D] = unit_value | (asserted_width_high   & 0x3F)
REG[0x6F] = unit_value | (deasserted_width_high & 0x3F)
```

- `unit_value`：佔 bit[7:6]（2 bits）
- `*_width_high`：佔 bit[5:0]（6 bits，最大值 63 / 0x3F）

### REG[0x6C] / REG[0x6E]（低位元暫存器）

```
REG[0x6C] = asserted_width_low   & 0xFF  （8 bits，最大 255）
REG[0x6E] = deasserted_width_low & 0xFF
```

### 14-bit 組合寬度（Combined Width）

參數的高 6 bits 與低 8 bits 可以組合成一個 14-bit 值：

```
combined_width = (width_high << 8) | width_low     // 0–16383

// 拆解
width_high = (combined_width >> 8) & 0x3F
width_low  =  combined_width       & 0xFF
```

### GP1 範例計算

- `asserted_width_unit = 2` → `unit_value = 0x80`（1ms）
- `asserted_width_high = 2`，`asserted_width_low = 10`
- `REG[0x6D] = 0x80 | 0x02 = 0x82`
- `REG[0x6C] = 0x0A`
- 實際斷言脈衝時間 = `(2 * 256 + 10) * 1ms = 522ms`

### GP2 範例計算

- `asserted_width_high = 3`，`asserted_width_low = 10`
- `REG[0x6D] = 0x80 | 0x03 = 0x83`
- `REG[0x6C] = 0x0A`
- 實際斷言脈衝時間 = `(3 * 256 + 10) * 1ms = 778ms`

> **注意**：實際脈衝時間 = `(high * 256 + low) * unit`，但確切的硬體計算方式需參閱 DM9051 Datasheet。

---

## 暫存器操作序列

### 輸出觸發完整序列（Output Trigger Setup）

```
步驟 1：確認 PTP 已啟用
  READ  REG[0x61]
  若 bit0 = 0，則 WRITE REG[0x61] = 0x01

步驟 2：選擇 GPIO 腳位
  WRITE REG[0x60] = (GPIO_pin == 1) ? 0x02 : 0x00

步驟 3：設定 GPIO 控制模式
  WRITE REG[0x6A] = 0x06 | is_input_mode
  // 0x04: 中斷啟用, 0x02: 觸發啟用, 0x01: 事件輸入模式

步驟 4：設定觸發模式與極性
  WRITE REG[0x6B] = trigger_values[trigger_mode]
                  | (trig_por ? 0x02 : 0x00)
  // trigger_values = {0x00, 0x04, 0x08, 0x0C}

步驟 5（僅脈衝模式）：設定脈衝寬度
  WRITE REG[0x6D] = unit_value | asserted_width_high
  WRITE REG[0x6C] = asserted_width_low
  WRITE REG[0x6F] = unit_value | deasserted_width_high
  WRITE REG[0x6E] = deasserted_width_low

步驟 6：讀取當前 PTP 時間
  WRITE REG[0x61] = 0x84   (RESET_IDX + READ_CLOCK)
  READ  REG[0x68] × 8 bytes → ts（當前時間）

步驟 7：計算觸發目標時間
  trigger_ts.seconds     = ts.seconds + sec_offset
  trigger_ts.nanoseconds = 0

步驟 8：寫入觸發目標時間
  WRITE REG[0x61] = 0x80   (RESET_IDX)
  WRITE REG[0x68] × 8 bytes（trigger_ts，LSB first）

步驟 9：啟動 GPIO 觸發
  WRITE REG[0x62] = 0x80 | (GPIO_pin ? 0x20 : 0x10)
  // 0x80: PTP 中斷啟用
  // 0x10: 啟動 GPIO0 / 0x20: 啟動 GPIO1
```

### 輸入事件讀取序列（Input Event Capture）

```
步驟 1：讀取中斷狀態
  READ  REG[0x60]
  檢查：GPIO0 → bit5(0x20), GPIO1 → bit7(0x80)

步驟 2：清除中斷旗標（Write-1-to-Clear）
  WRITE REG[0x60] = (GPIO0 ? 0x20 : 0x80)

步驟 3：啟動時間戳讀取
  WRITE REG[0x62] = (GPIO0 ? 0x10 : 0x20)

步驟 4：重置 REG[0x68] 索引
  WRITE REG[0x61] = 0x80

步驟 5：讀取事件時間戳
  READ  REG[0x68] × 8 bytes → event_ts
  ts.nanoseconds = buf[0..3]（LSB first）
  ts.seconds     = buf[4..7]（LSB first）
```

---

## `ptp_1588_gpio_both` 完整流程

```
ptp_1588_gpio_both()
├── 初始化 gpio_params_gp1（GPIO_pin=0, trigger_mode=2, sec_offset=1, ...）
├── 初始化 gpio_params_gp2（GPIO_pin=1, trigger_mode=2, sec_offset=1, ...）
│
├── print_gpio_trigger_params03(&gpio_params_gp1)  // 顯示/編輯 GP1 參數
├── print_gpio_trigger_params03(&gpio_params_gp2)  // 顯示/編輯 GP2 參數
│
├── test_1588_gpio_trigger04(&gpio_params_gp1)     // 設定並啟動 GP1 觸發
├── test_1588_gpio_trigger04(&gpio_params_gp2)     // 設定並啟動 GP2 觸發
│
└── while(1)
    ├── wait_for_gpio_event_or_trigger(&gp1, &ts_gp1)
    │   └── 若返回 TRUE：
    │       └── [輸出模式] 重新觸發 test_1588_gpio_trigger04(&gpio_params_gp1)
    │
    ├── wait_for_gpio_event_or_trigger(&gp2, &ts_gp2)
    │   └── 若返回 TRUE：
    │       └── [輸出模式] 重新觸發 test_1588_gpio_trigger04(&gpio_params_gp2)
    │
    └── 偵測 ESC 鍵（USART1 接收 0x27）→ break
```

---

## 移植注意事項

### 1. 硬體抽象層替換

原始碼依賴以下平台相關 API，移植時需替換為目標平台的實作：

| 原始函式 | 功能 | 替換建議 |
|---------|------|---------|
| `dm_rd_reg(usb_handle, reg, len, buf)` | 讀取 DM9051 暫存器 | SPI/USB/I2C 讀取函式 |
| `dm_wr_reg(usb_handle, reg, len, buf)` | 寫入 DM9051 暫存器 | SPI/USB/I2C 寫入函式 |
| `usart_get_received_data(USART1)` | 讀取 USART 接收資料 | 目標平台 UART 接收函式 |
| `sys_now()` | 取得系統毫秒時間 | HAL_GetTick() 或同等函式 |

### 2. `usb_handle` 全域變數

原始碼中的 `usb_handle` 為全域變數，對應 USB 通訊橋接器。  
移植時若使用 SPI 直連，可移除此參數或替換為 SPI CS 腳位控制。

### 3. `static` 區域變數的注意事項

`gpio_params_gp1` 和 `gpio_params_gp2` 宣告為 `static`，初始值只會設定一次。  
若需要在每次呼叫時重置參數，應移除 `static` 關鍵字。

### 4. 中斷驅動 vs. 輪詢

目前 `wait_for_gpio_event_or_trigger()` 以**輪詢**方式檢查 REG[0x60]。  
若目標平台支援硬體中斷，可改為在中斷服務程序 (ISR) 中設置旗標，再由主迴圈檢查。

### 5. 最小必要函式清單

移植此功能所需的最少函式集：

```
必要：
  - test_1588_gpio_trigger04()        // 核心：設定暫存器並啟動
  - wait_for_gpio_event_or_trigger()  // 核心：等待完成
  - dm9051a_ptp_get_time()            // 讀取 PTP 時鐘
  - test_1588_reg_68_wd01()           // 寫入觸發時間

可選（用於互動式設定）：
  - print_gpio_trigger_params03()     // 顯示/編輯參數介面
  - dm9051_gpio_event_read_ptp_clock() // 僅輸入模式時需要
```

### 6. 編譯依賴

```c
#include <stdint.h>
#include "ptp_types.h"          // ptp_time_t
#include "dm9051a_ptp_1588.h"   // 暫存器位址、ptp_clk_type_t
```

---

*本文件由 `ptp_functions.c` 原始碼分析自動整理，如有疑問請對照原始碼第 940–2497 行。*
