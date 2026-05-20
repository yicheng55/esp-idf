/**
 ******************************************************************************
 * @file     ptp_gpio_generate.c
 * @version  v1.0.0
 * @date     2026-05-06
 * @brief    DM9058 PTP 1588 GPIO Single Pulse output driver
 *
 * @details  Implements GP1 (GPIO0) and GP2 (GPIO1) single-pulse output using
 *           DM9058 PTP hardware trigger registers.
 *
 *           Register layout (from ptp_1588_gpio_both_porting_guide.md):
 *             REG[0x60] PSGR  - status / IRQ flags (W1C)
 *             REG[0x61] PCCR  - PTP clock control
 *             REG[0x62] PTPCR - PTP timer / GPIO trigger start
 *             REG[0x68] PSTR  - 8-byte time register (auto-increment index)
 *             REG[0x6A] PGCR  - GPIO control (mode / IRQ enable)
 *             REG[0x6B] PGTER - GPIO trigger type + polarity
 *             REG[0x6C/0x6D]  - asserted pulse width low/high+unit
 *             REG[0x6E/0x6F]  - deasserted pulse width low/high+unit
 *
 *           底層暫存器存取使用 HAL_write_reg() / HAL_read_reg()，
 *           定義於 utilities/dm9058_edriver_v1.6.1a/core/dm9058_constants.h。
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>

#include "ptp_gpio_generate.h"
#include "core/dm9058_constants.h"   /* HAL_write_reg / HAL_read_reg         */

/* 從 dm9058_ptp.c 匯出的 PTP 硬體重啟通知旗標（避免引入完整的 LwIP 依賴標頭）。
 * 定義於 dm9058_edriver_v1.6.1a/dm9058_edriver_extend/dm9058_ptp.c，
 * 宣告於 dm9058_ptp.h：extern volatile uint8_t g_dm9058_ptp_restarted。    */
extern volatile uint8_t g_dm9058_ptp_restarted;

/* ---------------------------------------------------------------------------
 * 模組私有資料
 * --------------------------------------------------------------------------- */

/* GP1 / GP2 通道參數（模組初始化後保留，供 update 重新觸發使用） */
static gpio_pulse_params_t s_gp1;
static gpio_pulse_params_t s_gp2;

/* GP1 功能開關：0=停用（不做 arm/rearm/timeout 輪詢），1=啟用 */
static const uint8_t s_gp1_enabled = 1u;
/* GP2 功能開關：0=停用（不做 arm/rearm/timeout 輪詢），1=啟用 */
static const uint8_t s_gp2_enabled = 0u;

/* 模組是否已初始化 */
static uint8_t s_initialized = 0;

/* 各通道最後一次 arm 的系統毫秒時間（用於超時保護） */
static uint32_t s_gp1_last_arm_tick = 0;
static uint32_t s_gp2_last_arm_tick = 0;

/* 觸發完成旗標已偵測，但尚未清除 / re-arm 的狀態。
 * REG[0x60] bit5/bit7 可能在 pulse 尚未完全輸出時就出現，不能同一輪立刻清除。 */
static uint8_t  s_gp1_done_pending = 0;
static uint8_t  s_gp2_done_pending = 0;
static uint32_t s_gp1_done_tick = 0;
static uint32_t s_gp2_done_tick = 0;

/* 最後一次偵測到 PTP restart 的系統毫秒時間（由 g_dm9058_ptp_restarted 觸發）。
 * slave 模式下 PTP servo 呼叫 dm9058_ptptime_settime() 時 REG[0x60]=0x01 soft reset，
 * 會清掉所有 GPIO trigger 設定；需等時鐘穩定後再 rearm，避免 arm→clear 的死循環。 */
static uint32_t s_last_ptp_restart_tick = 0;

/* ---------------------------------------------------------------------------
 * 私有輔助函式
 * --------------------------------------------------------------------------- */

/**
 * @brief  將 pulse asserted width 換算成 update() 使用的毫秒保護時間
 * @param  p  通道參數
 * @retval 至少 1ms 的 guard time
 */
static uint32_t gpio_pulse_asserted_guard_ms(const gpio_pulse_params_t *p)
{
    uint32_t count;
    uint32_t guard_ms;

    if (!p ||
        p->is_input_mode != GPIO_MODE_OUTPUT ||
        (p->trigger_mode != GPIO_TRIG_MODE_SINGLE &&
         p->trigger_mode != GPIO_TRIG_MODE_PERIODIC))
    {
        return 1u;
    }

    count = ((uint32_t)(p->pulse.asserted_high & 0x3Fu) << 8)
          | (uint32_t)p->pulse.asserted_low;
    if (count == 0u)
        count = 1u;

    switch (p->pulse.asserted_unit & 0xC0u)
    {
    case PULSE_UNIT_1MS:
        guard_ms = count;
        break;

    case PULSE_UNIT_1US:
    case 0xC0u: /* bit[7:6]=11 同樣視為 1us */
        guard_ms = (count + 999u) / 1000u;
        break;

    case PULSE_UNIT_120NS:
    default:
        guard_ms = ((count * 120u) + 999999u) / 1000000u;
        break;
    }

    /* 多留 1ms，避免 update tick 與硬體 pulse 邊界同時發生。 */
    return guard_ms + 1u;
}

/**
 * @brief  從 DM9058 連續讀取 REG[0x68] 8 bytes，取得當前 PTP 時間
 * @param  sec_out  輸出：PTP 秒數
 * @retval None
 *
 * @note   呼叫前需先寫入 REG[0x61] = 0x84 (RESET_IDX | READ_CLOCK)
 *         讀取格式：buf[0..3] = nanoseconds LSB first，buf[4..7] = seconds LSB first
 */
static uint32_t gpio_pulse_read_ptp_seconds(uint32_t *ns_out)
{
    uint8_t buf[8];
    int     i;

    /* 觸發讀取並重置 REG[0x68] 自動遞增索引 */
    HAL_write_reg(DM9058_REG_PCCR, 0x84);   /* RESET_IDX(0x80) | READ_CLOCK(0x04) */

    for (i = 0; i < 8; i++)
        buf[i] = HAL_read_reg(DM9058_REG_PSTR);

    /* buf[0..3] = nanoseconds，LSB first */
    if (ns_out)
        *ns_out = (uint32_t)buf[0]
                | ((uint32_t)buf[1] << 8)
                | ((uint32_t)buf[2] << 16)
                | ((uint32_t)buf[3] << 24);

    /* buf[4..7] = seconds，LSB first */
    return (uint32_t)buf[4]
         | ((uint32_t)buf[5] << 8)
         | ((uint32_t)buf[6] << 16)
         | ((uint32_t)buf[7] << 24);
}

/**
 * @brief  將觸發目標時間寫入 REG[0x68]（8 bytes，LSB first）
 * @param  seconds      目標觸發秒數
 * @param  nanoseconds  目標觸發奈秒數（通常為 0）
 * @retval None
 *
 * @note   呼叫前需先寫入 REG[0x61] = 0x80 (RESET_IDX)
 */
static void gpio_pulse_write_trigger_time(uint32_t seconds, uint32_t nanoseconds)
{
    int i;
    uint8_t buf[8];

    buf[0] = (uint8_t)(nanoseconds & 0xFF);
    buf[1] = (uint8_t)((nanoseconds >> 8)  & 0xFF);
    buf[2] = (uint8_t)((nanoseconds >> 16) & 0xFF);
    buf[3] = (uint8_t)((nanoseconds >> 24) & 0xFF);
    buf[4] = (uint8_t)(seconds & 0xFF);
    buf[5] = (uint8_t)((seconds >> 8)  & 0xFF);
    buf[6] = (uint8_t)((seconds >> 16) & 0xFF);
    buf[7] = (uint8_t)((seconds >> 24) & 0xFF);

    /* 重置 REG[0x68] 自動遞增索引 */
    HAL_write_reg(DM9058_REG_PCCR, 0x80);   /* RESET_IDX */

    for (i = 0; i < 8; i++)
        HAL_write_reg(DM9058_REG_PSTR, buf[i]);
}

/**
 * @brief  設定暫存器並啟動 GPIO 觸發（輸出）或啟用事件捕捉（輸入）
 * @param  p     通道參數指標
 * @param  tick  呼叫時的系統毫秒時間（用於超時保護計時）
 * @retval None
 *
 * 操作序列（依 ptp_1588_gpio_both_porting_guide.md 第「暫存器操作序列」節）：
 *   步驟 1  選擇 GPIO 腳位
 *   步驟 2  確認 PTP 已啟用
 *   步驟 3  設定 GPIO 控制模式（由 is_input_mode 決定輸出/輸入）
 *   步驟 4  設定觸發/事件模式 + 極性（由 trigger_mode / event_mode 決定）
 *   步驟 5  設定脈衝寬度（僅 trigger_mode = SINGLE / PERIODIC 時寫入）
 *   步驟 6  讀取當前 PTP 時間
 *   步驟 7  計算觸發目標時間（當前秒 + sec_offset）
 *   步驟 8  寫入觸發目標時間
 *   步驟 9  啟動 GPIO 觸發
 */
static void gpio_pulse_arm(gpio_pulse_params_t *p, uint32_t tick)
{
    /* REG[0x6B] 觸發模式查找表：索引 = trigger_mode (0–3) */
    static const uint8_t trig_mode_tbl[4] = {
        PGTER_MODE_EDGE,        /* GPIO_TRIG_MODE_EDGE     = 0 */
        PGTER_MODE_TOGGLE,      /* GPIO_TRIG_MODE_TOGGLE   = 1 */
        PGTER_MODE_SINGLE,      /* GPIO_TRIG_MODE_SINGLE   = 2 */
        PGTER_MODE_PERIODIC     /* GPIO_TRIG_MODE_PERIODIC = 3 */
    };

    uint32_t current_sec;
    uint32_t current_ns;
    uint32_t trigger_sec;
    uint8_t  pccr_val;
    uint8_t  pgcr_val;
    uint8_t  pgter_val;
    uint8_t  start_cmd;

    /* 步驟 1：確保 PTP 已啟用（REG[0x61] bit0 = 1）
     * ptpd 的 v51_adjust_ptp_frequency() 執行後會將 REG[0x61] 設為 0x60/0x20，
     * dm9058_ptptime_settime() 的 PTP restart 後也可能改變狀態，
     * 因此每次 arm 前無條件寫入 0x01 以確保一致性。 */
    pccr_val = HAL_read_reg(DM9058_REG_PCCR);
    HAL_write_reg(DM9058_REG_PCCR, 0x01u);
    GPIO_DBG("PTP enabled reg[0x61] = 0x%02X (was 0x%02X)\r\n", 0x01u, pccr_val);

    /* 步驟 2：選擇 GPIO 腳位（REG[0x60] bit1 = GPIO_pin） */
    {
        uint8_t psgr_val = p->gpio_pin ? 0x02u : 0x00u;
        HAL_write_reg(DM9058_REG_PSGR, psgr_val);
        GPIO_DBG("GPIO %u selected. Register 0x60 = 0x%02X\r\n",
                 p->gpio_pin, psgr_val);
    }

    /* 步驟 3：GPIO 控制模式
     *   REG[0x6A] = 0x04(IRQ enable) | 0x02(trigger enable) | is_input_mode(0x01)
     *   輸出：0x06，輸入：0x07 */
    pgcr_val = 0x06u | (p->is_input_mode & 0x01u);
    HAL_write_reg(DM9058_REG_PGCR, pgcr_val);
    GPIO_DBG("register 0x6A = 0x%02X\r\n", pgcr_val);

    /* 步驟 4：觸發/事件模式 + 極性
     *   輸出模式：trigger_mode 決定 REG[0x6B] bit[3:2]
     *   輸入模式：event_mode 決定邊緣方向（0=上升沿 / 1=下降沿） */
    if (p->is_input_mode == GPIO_MODE_OUTPUT)
    {
        uint8_t mode_idx = (p->trigger_mode < 4u) ? p->trigger_mode : GPIO_TRIG_MODE_SINGLE;
        pgter_val = trig_mode_tbl[mode_idx] | (p->trig_por ? PGTER_ACTIVE_HIGH : 0x00u);
    }
    else
    {
        /* 輸入模式：以 event_mode 選擇邊緣，極性位元同樣有效 */
        pgter_val = (p->event_mode ? 0x01u : 0x00u)
                  | (p->trig_por ? PGTER_ACTIVE_HIGH : 0x00u);
    }
    HAL_write_reg(DM9058_REG_PGTER, pgter_val);
    GPIO_DBG("register 0x6B = 0x%02X\r\n", pgter_val);
    GPIO_DBG("GPIO %u configured as %s with trigger mode %u, polarity: %s.\r\n",
             p->gpio_pin,
             p->is_input_mode ? "input" : "output",
             p->trigger_mode,
             p->trig_por ? "Active High" : "Active Low");

    /* 步驟 5：脈衝寬度設定（僅輸出 Single / Periodic 脈衝模式需要） */
    if (p->is_input_mode == GPIO_MODE_OUTPUT &&
        (p->trigger_mode == GPIO_TRIG_MODE_SINGLE ||
         p->trigger_mode == GPIO_TRIG_MODE_PERIODIC))
    {
        uint8_t pa_hw = (p->pulse.asserted_unit   & 0xC0u) | (p->pulse.asserted_high   & 0x3Fu);
        uint8_t pd_hw = (p->pulse.deasserted_unit & 0xC0u) | (p->pulse.deasserted_high & 0x3Fu);
        HAL_write_reg(DM9058_REG_PA_HW, pa_hw);
        HAL_write_reg(DM9058_REG_PA_LW, p->pulse.asserted_low);
        HAL_write_reg(DM9058_REG_PD_HW, pd_hw);
        HAL_write_reg(DM9058_REG_PD_LW, p->pulse.deasserted_low);
        GPIO_DBG("register 0x6D = 0x%02X\r\n", pa_hw);
        GPIO_DBG("register 0x6C = 0x%02X\r\n", p->pulse.asserted_low);
        GPIO_DBG("register 0x6F = 0x%02X\r\n", pd_hw);
        GPIO_DBG("register 0x6E = 0x%02X\r\n", p->pulse.deasserted_low);
    }

    /* 步驟 6：讀取當前 PTP 秒數 + 奈秒 */
    current_ns  = 0u;
    current_sec = gpio_pulse_read_ptp_seconds(&current_ns);
    GPIO_DBG("PTP Get Clock : 0x%08X_0x%08X, %08u.%09u\r\n",
             current_sec, current_ns, current_sec, current_ns);


    /* 步驟 7：觸發目標 = 當前秒 + 偏移量，奈秒歸零
     * 若當前奈秒 >= 500ms，額外多加 1 秒緩衝，避免觸發目標距當前時間不足而不穩定 */
    trigger_sec = current_sec + p->sec_offset;
    if (current_ns >= 500000000u)
        trigger_sec += 1u;

    /* 步驟 8：寫入觸發目標時間 */
    gpio_pulse_write_trigger_time(trigger_sec, 0u);
    GPIO_DBG("PTP Set Clock : 0x%08X_0x%08X, %08u.%09u\r\n",
             trigger_sec, 0u, trigger_sec, 0u);

    /* 步驟 9：啟動 GPIO 觸發（0x10 = GP1，0x20 = GP2）
     * 不設定 PTPCR_IRQ_ENABLE(0x80)，保持與 test_1588_gpio_trigger03 一致，
     * 避免與 ptpd 的 PTP IRQ 使用發生衝突。 */
    start_cmd = (p->gpio_pin ? PTPCR_GP2_TRIGGER : PTPCR_GP1_TRIGGER);
    HAL_write_reg(DM9058_REG_PTPCR, start_cmd);
    GPIO_DBG("GPIO %u trigger configured and loaded.\r\n", p->gpio_pin);
    GPIO_DBG("Waiting for trigger...\r\n");

    /* 更新超時計時基準 */
    if (p->gpio_pin == 0)
    {
        s_gp1_last_arm_tick = tick;
        s_gp1_done_pending = 0;
    }
    else
    {
        s_gp2_last_arm_tick = tick;
        s_gp2_done_pending = 0;
    }

    printf("[GPIO_PULSE] GP%u armed: mode=%s trigger_mode=%u trig_at_sec=%u (+%u s)\r\n",
           p->gpio_pin + 1u,
           p->is_input_mode ? "INPUT" : "OUTPUT",
           p->trigger_mode,
           trigger_sec, p->sec_offset);
}

/* ---------------------------------------------------------------------------
 * 公開 API 實作
 * --------------------------------------------------------------------------- */

/**
 * @brief  初始化 GP1 / GP2 並首次觸發 Single Pulse 輸出
 * @note   須在 PTP 時鐘已啟用後呼叫（PTP_STATE_RUNNING 進入後）
 * @retval None
 */
void ptp_gpio_generate_init(void)
{
    /* --- GP1 (GPIO0) 參數 -------------------------------------------------- */
    memset(&s_gp1, 0, sizeof(s_gp1));
    s_gp1.gpio_pin      = 0;                    /* GP1 = GPIO0                  */
    s_gp1.is_input_mode = GPIO_MODE_OUTPUT;     /* 輸出觸發模式                  */
    s_gp1.event_mode    = GPIO_EVENT_RISING;    /* 預留：上升沿（輸入模式用）     */
    s_gp1.trigger_mode  = GPIO_TRIG_MODE_SINGLE;/* Single Pulse                  */
    s_gp1.trig_por      = 1;                    /* Active High                   */
    s_gp1.sec_offset    = 1;                    /* 當前 PTP 時間 + 1 秒後觸發    */
    /* 斷言脈衝：unit=1ms, high=0, low=2 → (0*256+2)*1ms = 2 ms                */
    s_gp1.pulse.asserted_unit   = PULSE_UNIT_1MS;
    s_gp1.pulse.asserted_high   = 0;
    s_gp1.pulse.asserted_low    = 2;
    /* 去斷言脈衝：unit=1ms, high=2, low=20 → (2*256+20)*1ms = 532 ms          */
    s_gp1.pulse.deasserted_unit = PULSE_UNIT_1MS;
    s_gp1.pulse.deasserted_high = 2;
    s_gp1.pulse.deasserted_low  = 20;

    /* --- GP2 (GPIO1) 參數 -------------------------------------------------- */
    memset(&s_gp2, 0, sizeof(s_gp2));
    s_gp2.gpio_pin      = 1;                    /* GP2 = GPIO1                  */
    s_gp2.is_input_mode = GPIO_MODE_OUTPUT;     /* 輸出觸發模式                  */
    s_gp2.event_mode    = GPIO_EVENT_RISING;    /* 預留：上升沿（輸入模式用）     */
    s_gp2.trigger_mode  = GPIO_TRIG_MODE_SINGLE;/* Single Pulse                  */
    s_gp2.trig_por      = 1;                    /* Active High                   */
    s_gp2.sec_offset    = 1;                    /* 當前 PTP 時間 + 1 秒後觸發    */
    /* 斷言脈衝：unit=1ms, high=0, low=2 → (0*256+2)*1ms = 2 ms                */
    s_gp2.pulse.asserted_unit   = PULSE_UNIT_1MS;
    s_gp2.pulse.asserted_high   = 0;
    s_gp2.pulse.asserted_low    = 2;
    /* 去斷言脈衝：unit=1ms, high=2, low=20 → (2*256+20)*1ms = 532 ms          */
    s_gp2.pulse.deasserted_unit = PULSE_UNIT_1MS;
    s_gp2.pulse.deasserted_high = 2;
    s_gp2.pulse.deasserted_low  = 20;

    printf("[GPIO_PULSE] Initializing GP1/GP2 single pulse output...\r\n");

    if (s_gp1_enabled)
        gpio_pulse_arm(&s_gp1, 0);
    if (s_gp2_enabled)
        gpio_pulse_arm(&s_gp2, 0);

    s_initialized = 1;

    printf("[GPIO_PULSE] GP1/GP2 single pulse initialized\r\n");
}

/**
 * @brief  在主迴圈中定期呼叫，輪詢觸發完成旗標並自動重新觸發
 * @param  tick  系統毫秒計數（通常傳入 local_time）
 * @retval None
 *
 * 邏輯：
 *   1. 讀取 REG[0x60] 狀態暫存器
 *   2. 若偵測到 PTP restart（slave 校時路徑），記錄時間戳並重置超時計時器；
 *      等待 GPIO_POST_RESTART_GUARD_MS 無再次 restart 後，才由超時保護重新 arm。
 *      （立刻 arm 會因 slave 仍在校時而被再次 PTP restart 清掉，造成 arm→clear 死循環。）
 *   3. 若 GP1/GP2 完成旗標置位 → 先記錄 pending，等待 asserted pulse guard
 *   4. guard 到期後才 W1C 清除旗標，並重新 arm 對應 GPIO
 *   5. 超時保護：若距上次 arm 超過 GPIO_PULSE_REARM_TIMEOUT_MS，且距上次 PTP restart
 *      超過 GPIO_POST_RESTART_GUARD_MS，強制重新 arm
 */
void ptp_gpio_generate_update(uint32_t tick)
{
    uint8_t status;
    uint8_t rearm_gp1 = 0;
    uint8_t rearm_gp2 = 0;
    uint32_t guard_ms;

    if (!s_initialized)
        return;

    /* PTP 硬體復位偵測：
     * slave 模式下 ptpd servo 呼叫 dm9058_ptptime_settime()（setTime / updateTime），
     * 會執行 REG[0x60]=0x01 soft reset，清掉所有 GPIO trigger 設定（包含 pulse 正在輸出時）。
     * 若立刻重新 arm，slave 仍在校時期間可能再次觸發 setTime，造成 arm→clear 死循環。
     * 正確做法：記錄 restart 時間，重置超時計時器，等待 GPIO_POST_RESTART_GUARD_MS 穩定後
     * 再由超時保護路徑 arm，確保 PTP 時鐘已穩定再安排觸發時間。 */
    if (g_dm9058_ptp_restarted)
    {
        g_dm9058_ptp_restarted = 0;
        s_last_ptp_restart_tick = tick;
        printf("[GPIO_PULSE] PTP restart detected (tick=%u), rearm deferred by %u ms\r\n",
               tick, (unsigned)GPIO_POST_RESTART_GUARD_MS);
        if (s_gp1_enabled)
        {
            s_gp1_done_pending = 0;
            s_gp1_last_arm_tick = tick; /* 重置超時計時器：從現在起算 REARM_TIMEOUT_MS */
        }
        if (s_gp2_enabled)
        {
            s_gp2_done_pending = 0;
            s_gp2_last_arm_tick = tick;
        }
        return;
    }

    /* 讀取 PTP 系統狀態暫存器 */
    status = HAL_read_reg(DM9058_REG_PSGR);

    /* GP1 觸發完成旗標（bit5 = 0x20） */
    if (s_gp1_enabled && !s_gp1_done_pending && (status & GPIO0_TRIGGER_DONE_BIT))
    {
        GPIO_DBG("Trigger output... (GP1), wait pulse guard before clear\r\n");
        s_gp1_done_pending = 1;
        s_gp1_done_tick = tick;
    }

    if (s_gp1_enabled && s_gp1_done_pending)
    {
        guard_ms = gpio_pulse_asserted_guard_ms(&s_gp1);
        if ((tick - s_gp1_done_tick) >= guard_ms)
        {
            /* Write-1-to-Clear：清除 GP1 旗標 */
            HAL_write_reg(DM9058_REG_PSGR, GPIO0_TRIGGER_DONE_BIT);
            rearm_gp1 = 1;
            s_gp1_done_pending = 0;
            GPIO_DBG("GP1 trigger done cleared after %u ms guard\r\n", guard_ms);
        }
    }

    /* GP2 觸發完成旗標（bit7 = 0x80） */
    if (s_gp2_enabled && !s_gp2_done_pending && (status & GPIO1_TRIGGER_DONE_BIT))
    {
        GPIO_DBG("Trigger output... (GP2), wait pulse guard before clear\r\n");
        s_gp2_done_pending = 1;
        s_gp2_done_tick = tick;
    }

    if (s_gp2_enabled && s_gp2_done_pending)
    {
        guard_ms = gpio_pulse_asserted_guard_ms(&s_gp2);
        if ((tick - s_gp2_done_tick) >= guard_ms)
        {
            /* Write-1-to-Clear：清除 GP2 旗標 */
            HAL_write_reg(DM9058_REG_PSGR, GPIO1_TRIGGER_DONE_BIT);
            rearm_gp2 = 1;
            s_gp2_done_pending = 0;
            GPIO_DBG("GP2 trigger done cleared after %u ms guard\r\n", guard_ms);
        }
    }

    /* 超時保護：防止旗標遺失導致通道停止輸出。
     * 同時加入 post-restart guard：若距上次 PTP restart 未超過 GPIO_POST_RESTART_GUARD_MS，
     * 不執行 rearm，確保 slave 時鐘校正期間 GPIO 不會不斷被重設截斷。 */
    if (s_gp1_enabled && !s_gp1_done_pending && !rearm_gp1 &&
        ((tick - s_gp1_last_arm_tick) >= GPIO_PULSE_REARM_TIMEOUT_MS) &&
        ((tick - s_last_ptp_restart_tick) >= GPIO_POST_RESTART_GUARD_MS))
    {
        printf("[GPIO_PULSE] GP1 timeout rearm (tick=%u, last_arm=%u, last_restart=%u)\r\n",
               tick, s_gp1_last_arm_tick, s_last_ptp_restart_tick);
        rearm_gp1 = 1;
    }

    if (s_gp2_enabled && !s_gp2_done_pending && !rearm_gp2 &&
        ((tick - s_gp2_last_arm_tick) >= GPIO_PULSE_REARM_TIMEOUT_MS) &&
        ((tick - s_last_ptp_restart_tick) >= GPIO_POST_RESTART_GUARD_MS))
    {
        printf("[GPIO_PULSE] GP2 timeout rearm (tick=%u, last_arm=%u, last_restart=%u)\r\n",
               tick, s_gp2_last_arm_tick, s_last_ptp_restart_tick);
        rearm_gp2 = 1;
    }

    /* 重新觸發 */
    if (s_gp1_enabled && rearm_gp1)
        gpio_pulse_arm(&s_gp1, tick);

    if (s_gp2_enabled && rearm_gp2)
        gpio_pulse_arm(&s_gp2, tick);
}
