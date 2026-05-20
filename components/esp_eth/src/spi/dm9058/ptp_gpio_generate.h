/**
 ******************************************************************************
 * @file     ptp_gpio_generate.h
 * @version  v1.1.0
 * @date     2026-05-06
 * @brief    DM9058 PTP 1588 GPIO signal generation driver
 *
 * @details  Provides GP1 (GPIO0) and GP2 (GPIO1) output/input control
 *           using DM9058 PTP hardware trigger registers.
 *
 *           Output trigger modes (trigger_mode):
 *             GPIO_TRIG_MODE_EDGE     - single edge output
 *             GPIO_TRIG_MODE_TOGGLE   - toggle on each trigger
 *             GPIO_TRIG_MODE_SINGLE   - single pulse (default)
 *             GPIO_TRIG_MODE_PERIODIC - periodic pulse
 *
 *           Input event mode (event_mode, valid when is_input_mode = 1):
 *             GPIO_EVENT_RISING  - capture rising edge
 *             GPIO_EVENT_FALLING - capture falling edge
 *
 *           Call ptp_gpio_generate_init() once after PTP daemon starts, then
 *           call ptp_gpio_generate_update() every iteration of the main loop.
 ******************************************************************************
 */

#ifndef __PTP_GPIO_GENERATE_H
#define __PTP_GPIO_GENERATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * DM9058 PTP GPIO 暫存器位址
 * --------------------------------------------------------------------------- */
#define DM9058_REG_PSGR   0x60   /* PTP System General Register (status/IRQ) */
#define DM9058_REG_PCCR   0x61   /* PTP Clock Control Register               */
#define DM9058_REG_PTPCR  0x62   /* PTP Timer Control Register (start GPIO)  */
#define DM9058_REG_PSTR   0x68   /* PTP System Time Register (8 bytes, auto-inc) */
#define DM9058_REG_PGCR   0x6A   /* PTP GPIO Control Register                */
#define DM9058_REG_PGTER  0x6B   /* PTP GPIO Trigger Edge Register           */
#define DM9058_REG_PA_LW  0x6C   /* Asserted Pulse Width Low  (8-bit)        */
#define DM9058_REG_PA_HW  0x6D   /* Asserted Pulse Width High + Unit (8-bit) */
#define DM9058_REG_PD_LW  0x6E   /* Deasserted Pulse Width Low  (8-bit)      */
#define DM9058_REG_PD_HW  0x6F   /* Deasserted Pulse Width High + Unit (8-bit) */

/* ---------------------------------------------------------------------------
 * REG[0x60] 觸發完成旗標位元（Write-1-to-Clear）
 * --------------------------------------------------------------------------- */
#define GPIO0_TRIGGER_DONE_BIT  (1u << 5)   /* bit5: GP1 (GPIO0) 觸發完成 */
#define GPIO1_TRIGGER_DONE_BIT  (1u << 7)   /* bit7: GP2 (GPIO1) 觸發完成 */

/* ---------------------------------------------------------------------------
 * REG[0x62] 啟動觸發命令位元
 * --------------------------------------------------------------------------- */
#define PTPCR_IRQ_ENABLE    0x80   /* PTP 中斷啟用                */
#define PTPCR_GP1_TRIGGER   0x10   /* 啟動 GPIO0 (GP1) 觸發       */
#define PTPCR_GP2_TRIGGER   0x20   /* 啟動 GPIO1 (GP2) 觸發       */

/* ---------------------------------------------------------------------------
 * REG[0x6B] 觸發模式查找表值（bit[3:2]）
 * --------------------------------------------------------------------------- */
#define PGTER_MODE_EDGE     0x00   /* Edge 邊緣觸發               */
#define PGTER_MODE_TOGGLE   0x04   /* Toggle 翻轉觸發             */
#define PGTER_MODE_SINGLE   0x08   /* Single Pulse 單脈衝         */
#define PGTER_MODE_PERIODIC 0x0C   /* Periodic Pulse 週期脈衝     */
#define PGTER_ACTIVE_HIGH   0x02   /* Active High 極性（bit1）    */

/* ---------------------------------------------------------------------------
 * GPIO 工作模式（is_input_mode）
 * --------------------------------------------------------------------------- */
#define GPIO_MODE_OUTPUT    0u     /* 輸出觸發模式                */
#define GPIO_MODE_INPUT     1u     /* 輸入事件捕捉模式            */

/* ---------------------------------------------------------------------------
 * 輸出觸發類型（trigger_mode）
 * --------------------------------------------------------------------------- */
#define GPIO_TRIG_MODE_EDGE     0u /* REG[0x6B] = PGTER_MODE_EDGE     */
#define GPIO_TRIG_MODE_TOGGLE   1u /* REG[0x6B] = PGTER_MODE_TOGGLE   */
#define GPIO_TRIG_MODE_SINGLE   2u /* REG[0x6B] = PGTER_MODE_SINGLE   */
#define GPIO_TRIG_MODE_PERIODIC 3u /* REG[0x6B] = PGTER_MODE_PERIODIC */

/* ---------------------------------------------------------------------------
 * 輸入事件邊緣方向（event_mode，僅 is_input_mode=1 時有效）
 * --------------------------------------------------------------------------- */
#define GPIO_EVENT_RISING   0u     /* 上升沿觸發事件              */
#define GPIO_EVENT_FALLING  1u     /* 下降沿觸發事件              */

/* ---------------------------------------------------------------------------
 * 脈衝寬度單位（*_width_unit 對應 REG[0x6D/0x6F] bit[7:6]）
 * REG[0x6F] bit[7:6]: 00=120ns, 01=1us, 10=1ms, 11=1us
 * --------------------------------------------------------------------------- */
#define PULSE_UNIT_120NS    0x00   /* 120 ns / 計數  (bit[7:6]=00) */
#define PULSE_UNIT_1US      0x40   /* 1 µs   / 計數  (bit[7:6]=01) */
#define PULSE_UNIT_1MS      0x80   /* 1 ms   / 計數  (bit[7:6]=10) */
/* PULSE_UNIT_1US_ALT 0xC0 — bit[7:6]=11, 同樣為 1us（保留備用） */

/* ---------------------------------------------------------------------------
 * 超時保護：若超過此毫秒數仍未收到完成旗標，強制重新觸發
 * --------------------------------------------------------------------------- */
/* 超時保護時間設定原則：sec_offset * 1000 + 500 ms
 * 預設 sec_offset=1 → 1500ms。若修改 sec_offset 需同步調整此值。 */
#define GPIO_PULSE_REARM_TIMEOUT_MS  3000u

/* PTP restart 後的穩定等待時間（毫秒）。
 * slave 模式下 ptpd servo 校時路徑（setTime / updateTime）會呼叫
 * dm9058_ptptime_settime()，執行 REG[0x60]=0x01 soft reset 清掉所有 GPIO trigger 設定。
 * 若立刻重新 arm 而 PTP 仍不穩定，下次 setTime 會再次清掉設定，造成 arm→clear 死循環。
 * 設定原則：至少 >= GPIO_PULSE_REARM_TIMEOUT_MS，確保超時保護在 restart 停止後才觸發。
 * 建議值：3000ms（與 REARM_TIMEOUT_MS 相同）；slave 收斂通常需數秒至數十秒。 */
#define GPIO_POST_RESTART_GUARD_MS   3000u

/* ---------------------------------------------------------------------------
 * Debug output compile switch — 注解掉下一行以關閉 GPIO debug 輸出
 * --------------------------------------------------------------------------- */
//#define GPIO_PULSE_DEBUG

#ifdef GPIO_PULSE_DEBUG
#  include <stdio.h>
#  define GPIO_DBG(...)  printf(__VA_ARGS__)
#else
#  define GPIO_DBG(...)  do {} while(0)
#endif

/* ---------------------------------------------------------------------------
 * gpio_pulse_params_t — 單一 GPIO 通道完整參數
 * --------------------------------------------------------------------------- */
typedef struct
{
    uint8_t gpio_pin;        /* 腳位選擇：0 = GP1 (GPIO0), 1 = GP2 (GPIO1)          */
    uint8_t is_input_mode;   /* 工作模式：GPIO_MODE_OUTPUT(0) / GPIO_MODE_INPUT(1)   */
    uint8_t event_mode;      /* 輸入事件邊緣：GPIO_EVENT_RISING(0) / FALLING(1)      */
                             /* 僅 is_input_mode = GPIO_MODE_INPUT 時有效            */
    uint8_t trigger_mode;    /* 輸出觸發類型：GPIO_TRIG_MODE_EDGE/TOGGLE/SINGLE/PERIODIC */
                             /* 僅 is_input_mode = GPIO_MODE_OUTPUT 時有效           */
    uint8_t trig_por;        /* 觸發極性：0 = Active Low, 1 = Active High            */
    uint8_t sec_offset;      /* 觸發目標時間 = 當前 PTP 秒 + sec_offset               */

    struct {
        uint8_t asserted_unit;   /* 斷言脈衝單位：PULSE_UNIT_* 之一                  */
        uint8_t asserted_high;   /* 斷言脈衝高位寬度（bit[5:0], 0–63）               */
        uint8_t asserted_low;    /* 斷言脈衝低位寬度（8-bit, 0–255）                 */
        uint8_t deasserted_unit; /* 去斷言脈衝單位：PULSE_UNIT_* 之一                */
        uint8_t deasserted_high; /* 去斷言脈衝高位寬度（bit[5:0], 0–63）             */
        uint8_t deasserted_low;  /* 去斷言脈衝低位寬度（8-bit, 0–255）               */
    } pulse;                 /* 脈衝寬度參數（trigger_mode = SINGLE / PERIODIC 時有效） */
} gpio_pulse_params_t;

/* ---------------------------------------------------------------------------
 * 公開 API
 * --------------------------------------------------------------------------- */

/**
 * @brief  初始化 GP1 / GP2 並首次觸發 Single Pulse 輸出
 * @note   須在 PTP 時鐘已啟用後呼叫（PTP_STATE_RUNNING 進入後）
 * @retval None
 */
void ptp_gpio_generate_init(void);


/**
 * @brief  在主迴圈中定期呼叫，輪詢觸發完成旗標並自動重新觸發
 * @param  tick  系統毫秒計數（通常傳入 local_time）
 * @retval None
 */
void ptp_gpio_generate_update(uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif /* __PTP_GPIO_GENERATE_H */
