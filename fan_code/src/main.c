#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "..\inc\key.h"
#include "..\inc\motor.h"
#include "..\inc\adc_bat.h"
#include "intrins.h"

/* ======== 系统状态枚举 ======== */
typedef enum {
    STATE_SLEEP    = 0,   /* 低功耗休眠 */
    STATE_CHARGING = 1,   /* 充电中，蓝灯慢闪 */
    STATE_RUNNING  = 2,   /* 电机运行/待机 */
} state_t;

/* ======== 全局变量 ======== */
bit B_start = 0;
unsigned char throttle_index = 0;
unsigned int  motor_time     = 0;

static state_t       state         = STATE_SLEEP;
static bit           tick_10ms     = 0;
static unsigned char data pwm_value  = 0;
static unsigned char data last_pwm   = 0;

/* 低功耗状态 */
static bit           is_sleeping    = 0; /* 外设已关闭标志 */
static unsigned char data sleep_key_cnt = 0; /* 睡眠中长按计数（×100ms） */
static unsigned char data chg_blink_cnt = 0; /* 充电 LED 闪烁计数 */

/* 自动关机阈值 */
#define MOTOR_RUNTIME_TH    30000   /* 运行时 5min  */
#define MOTOR_IDLETIME_TH   3000    /* 空档时 30s   */

/* 前向声明 */
void init(void);
void clk_init(void);
void timer2_init(void);
void key_handle(void);
void led_show_throttle(void);
static void sleep_enter(void);
static void sleep_exit(void);

void bat_low_beebee(void) {}

/* ======================================================
 * 低功耗：关闭外设，启动掉电唤醒定时器
 * ====================================================== */
static void sleep_enter(void)
{
    /* 停止 Timer2（主节拍 10ms） */
    AUXR &= ~0x10;          /* T2R = 0：停止计数 */
    IE2  &= ~0x04;          /* ET2 = 0：禁止中断 */

    /* 停止 Timer0 / Timer1（电机换相 / BEMF 测量） */
    TR0 = 0;  ET0 = 0;
    TR1 = 0;  ET1 = 0;

    /* 关闭 ADC 电源（ADC_CONTR[7] = 0） */
    ADC_CONTR &= ~0x80;

    /* 关闭比较器（CMPCR1[7] CMPEN = 0） */
    CMPCR1 = 0x00;

    /* 配置掉电唤醒定时器，约 100ms
     * 振荡器 32KHz，周期 = (N+1) / 32000
     * N = 3199 = 0x0C7F → 3200 / 32000 = 100ms */
    WKTCL = 0x7F;
    WKTCH = 0x8C;           /* WKTEN=1 | 高4位 = 0x0C */

    /* WDT 最大预分频（PS=7，约 22.5ms @35MHz）
     * 掉电期间 WDT 暂停，WKTR 每 100ms 唤醒后及时喂狗 */
    WDT_CONTR = (WDT_CONTR & 0xF8) | 0x07;
}

/* ======================================================
 * 低功耗：恢复外设，停止掉电唤醒定时器
 * ====================================================== */
static void sleep_exit(void)
{
    /* 关闭掉电唤醒定时器 */
    WKTCH = 0x00;

    /* 重启 Timer2（主节拍 10ms） */
    timer2_init();

    /* 重新初始化 ADC（BEMF 比较器正输入，通道 13） */
    ADC_CONTR = 0x80 | 13;
    ADCCFG    = 0x21;
    P_SW2    |= 0x80;
    ADCTIM    = 0x20 + 20;

    /* 重新初始化比较器 */
    CMPCR1 = 0x8C;
    CMPCR2 = 60;

    /* 重置按键状态机（清除睡眠期间残留状态） */
    key_init();
}

/* ======================================================
 * 主函数
 * ====================================================== */
void main(void)
{
    init();

    /* 上电绿灯双闪：表明 MCU 正常启动 */
    led_g_on();  delay_ms(150);
    led_g_off(); delay_ms(100);
    led_g_on();  delay_ms(150);
    led_g_off();

    while (1)
    {
        /* ============================================================
         * 低功耗模式（SLEEP / CHARGING 均在此循环，掉电唤醒定时器驱动）
         * ============================================================ */
        if (state == STATE_SLEEP || state == STATE_CHARGING)
        {
            if (!is_sleeping)
            {
                /* 首次进入：停机、关外设 */
                is_sleeping   = 1;
                sleep_key_cnt = 0;
                chg_blink_cnt = 0;
                if (B_run || B_start) { motor_stop(); B_start = 0; }
                throttle_index = 0;
                led_all_off();
                POWER_SWITCH = 0;
                sleep_enter();
            }

            /* 进入掉电模式，WKTR 每 100ms 唤醒一次 */
            PCON |= 0x02;
            _nop_(); _nop_();
            /* ---- WKTR 溢出，MCU 从此处恢复执行 ---- */

            /* ① 喂狗（WDT 在掉电期间暂停，唤醒后立即喂） */
            WDT_CONTR |= 0x10;

            /* ② 长按检测：10 次 × 100ms = 1s */
            if (KEY_PIN == 0)
            {
                if (++sleep_key_cnt >= 10)
                {
                    sleep_key_cnt = 0;
                    if (VBUS_SENSE != 0)    /* 无充电线，允许开机 */
                    {
                        is_sleeping = 0;
                        sleep_exit();
                        POWER_SWITCH = 1;
                        pwm_value = 0;  last_pwm = 0;
                        motor_time = 0; throttle_index = 0;
                        state = STATE_RUNNING;
                        led_all_off();
                        led_r_on();     /* 红灯：0 档待机 */
                    }
                    /* else: 充电中，长按无效，保持当前状态 */
                }
            }
            else
            {
                sleep_key_cnt = 0;
            }

            /* ③ 充电线插拔检测 */
            if (state == STATE_SLEEP && VBUS_SENSE == 0)
            {
                /* 休眠时插入充电线 → 显示充电指示 */
                state = STATE_CHARGING;
                chg_blink_cnt = 0;
            }
            if (state == STATE_CHARGING && VBUS_SENSE != 0)
            {
                /* 充电线拔出 → 返回纯休眠 */
                state = STATE_SLEEP;
                led_all_off();
            }

            /* ④ 充电指示灯：蓝灯 500ms 亮 / 500ms 灭 */
            if (state == STATE_CHARGING)
            {
                if (++chg_blink_cnt >= 10) chg_blink_cnt = 0;
                led_all_off();
                if (chg_blink_cnt < 5) led_b_on();
            }

            continue;   /* 回到 while(1) 顶部 */
        }

        /* ============================================================
         * 正常运行模式：等待 Timer2 产生的 10ms 节拍
         * ============================================================ */
        if (!tick_10ms)
            continue;
        tick_10ms = 0;

        key_handle();
        bat_update_10ms();

        /* key_handle() 可能已将状态切回 SLEEP/CHARGING */
        if (state != STATE_RUNNING)
            continue;

        /* ---- 运行状态处理 ---- */
        {
            unsigned char ramp_max;
            char delta;

            /* ① 充电线插入：立即停机 */
            if (VBUS_SENSE == 0)
            {
                motor_stop();
                throttle_index = 0;
                state = STATE_CHARGING;
                continue;
            }

            /* ② 低压保护：蓝灯闪 4 次后休眠 */
            if (bat_is_low())
            {
                unsigned char blink;
                motor_stop();
                throttle_index = 0;
                bat_low_beebee();
                for (blink = 0; blink < 4; blink++)
                {
                    led_all_off();
                    led_b_on(); delay_ms(250);
                    led_all_off(); delay_ms(250);
                }
                state = STATE_SLEEP;
                continue;
            }

            /* ③ 电机控制 */
            if (B_run)
            {
                motor_update_filter();

                /* 失同步恢复：油门减半，重置计数 */
                if (desync_flag)
                {
                    desync_flag      = 0;
                    pwm_value        = pwm_value >> 1;
                    zero_crosses     = 0;
                    motor_stuck_time = 0;
                }

                /* 一阶低通滤波 */
                pwm_value = (unsigned char)(
                    ((unsigned int)pwm_value * 9 + THROTTLE[throttle_index]) / 10
                );

                /* 变化率钳位（防失步） */
                ramp_max = (phase_time > 500) ? PWM_RAMP_SLOW : PWM_RAMP_FAST;
                delta    = (char)pwm_value - (char)last_pwm;
                if      (delta >  (char)ramp_max) pwm_value = last_pwm + ramp_max;
                else if (delta < -(char)ramp_max) pwm_value = last_pwm - ramp_max;
                last_pwm = pwm_value;

                motor_set_pwm(pwm_value);

                /* 堵转超时 */
                if (++motor_stuck_time >= MOTOR_STUCK_TIMEOUT)
                {
                    motor_stop();
                    throttle_index = 0;
                }
            }
            else if (B_start)
            {
                /* B_start 必须在 motor_start() 执行期间保持为 1，
                 * motor_step() 依赖它来压制 BEMF 比较器中断。 */
                motor_start();
                B_start = 0;
                motor_enter_run();
                delay_ms(250);
                delay_ms(250);
                motor_stuck_time = 0;
                last_pwm  = PWM_START_VALUE;
                pwm_value = PWM_START_VALUE;
            }

            /* ④ 自动关机计时 */
            if (++motor_time >= (throttle_index == 0 ? MOTOR_IDLETIME_TH : MOTOR_RUNTIME_TH))
            {
                motor_time = 0;
                motor_stop();
                throttle_index = 0;
                state = STATE_SLEEP;
                continue;
            }

            led_show_throttle();
        }
    }
}

/* ======================================================
 * 按键事件处理（仅在运行模式 tick 路径中调用）
 * ====================================================== */
void key_handle(void)
{
    unsigned char evt = key_get_event();
    if (evt == KEY_EVT_NONE)
        return;

    motor_time = 0;

    switch (evt)
    {
    case KEY_EVT_SHORT:
        if (state != STATE_RUNNING)
            break;
        if (bat_is_low()) { bat_low_beebee(); break; }
        if (++throttle_index >= THROTTLE_SUM)
        {
            throttle_index = 0;
            motor_stop();
        }
        if (throttle_index == 1)
            B_start = 1;
        break;

    case KEY_EVT_LONG:
        if (state == STATE_RUNNING)
        {
            motor_stop();
            throttle_index = 0;
            state = STATE_SLEEP;
        }
        /* SLEEP/CHARGING 下的长按由低功耗循环直接检测，不经此函数 */
        break;

    case KEY_EVT_DOUBLE:
        if (state == STATE_RUNNING)
        {
            motor_stop();
            throttle_index = 0;
        }
        break;
    }
}

/* ======================================================
 * LED 档位显示
 * ====================================================== */
void led_show_throttle(void)
{
    static unsigned char last = 0xFF;
    if (throttle_index == last)
        return;
    last = throttle_index;

    led_all_off();
    switch (throttle_index)
    {
        case 0: led_r_on(); break;                          /* 待机：红 */
        case 1: led_g_on(); break;                          /* 1档：绿 */
        case 2: led_b_on(); break;                          /* 2档：蓝 */
        case 3: led_r_on(); led_g_on(); led_b_on(); break;  /* 3档：白 */
        default: break;
    }
}

/* ======================================================
 * 初始化
 * ====================================================== */
void init(void)
{
    clk_init();
    led_init();
    key_init();
    motor_init();
    timer2_init();
    bat_init();
    P_SW2 |= 0x80;   /* 保持 xdata SFR 访问使能（PWMA 寄存器需要） */
    EA = 1;
}

/* ======================================================
 * 时钟初始化（35MHz 内部 RC）
 * ====================================================== */
void clk_init(void)
{
    P_SW2   = 0x80;
    CLKDIV  = 0x04;
    IRTRIM  = T35M_ROMADDR;
    VRTRIM  = VRT35M_ROMADDR;
    IRCBAND = 0x01;
    CLKDIV  = 0x00;
    P_SW2   = 0x00;
}

/* ======================================================
 * Timer2：10ms 节拍（35MHz，12T）
 * ====================================================== */
void timer2_init(void)
{
    AUXR &= 0xFB;
    T2L   = 0x11;
    T2H   = 0x8E;
    IE2  |= 0x04;
    AUXR |= 0x10;
}

void TM2_Isr(void) interrupt 12
{
    WDT_CONTR |= 0x10;   /* 运行期间喂狗（10ms 间隔） */
    tick_10ms = 1;
    key_scan();
}
