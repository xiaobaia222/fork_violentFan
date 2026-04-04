#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "..\inc\key.h"
#include "..\inc\motor.h"
#include "..\inc\adc_bat.h"
#include "intrins.h"

/* ======== 系统状态枚举 ======== */
typedef enum {
    STATE_SLEEP    = 0,   /* 低功耗休眠，等待长按唤醒 */
    STATE_CHARGING = 1,   /* 充电中（VBUS 接入），蓝灯慢闪 */
    STATE_RUNNING  = 2,   /* 电机运行/待机 */
} state_t;

/* ======== 全局变量 ======== */
bit B_start = 0;
unsigned char throttle_index = 0;
unsigned int  motor_time     = 0;

static state_t        state      = STATE_SLEEP;
static bit            tick_10ms  = 0;
static unsigned char  pwm_value  = 0;
static unsigned char  last_pwm   = 0;

/* 自动关机阈值 */
#define MOTOR_RUNTIME_TH    30000   /* 运行时 5min */
#define MOTOR_IDLETIME_TH   3000    /* 空档时 30s  */

/* 前向声明 */
void init(void);
void clk_init(void);
void timer2_init(void);
void key_handle(void);
void led_show_throttle(void);

void bat_low_beebee(void)
{
}

/* ======== 主函数 ======== */
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
        /* 等待 Timer2 产生的 10ms 节拍 */
        if (!tick_10ms)
            continue;
        tick_10ms = 0;

        /* 每 tick 统一调用：与 ISR 中 key_scan() 同步执行 */
        key_handle();
        bat_update_10ms();

        /* ======== 状态机 ======== */
        switch (state)
        {
        /* ---- 休眠 ---- */
        case STATE_SLEEP:
            if (B_run || B_start)
            {
                motor_stop();
                B_start = 0;
            }
            throttle_index = 0;
            led_all_off();
            POWER_SWITCH = 0;
            PCON |= 0x01;   /* IDL 低功耗，Timer2 保持运行 */
            break;

        /* ---- 充电 ---- */
        case STATE_CHARGING:
        {
            static unsigned char chg_tick = 0;
            if (++chg_tick >= 100) chg_tick = 0;
            led_all_off();
            if (chg_tick < 50) led_b_on();   /* 蓝灯 500ms 亮/500ms 灭 */
            POWER_SWITCH = 0;
            PCON |= 0x01;
            break;
        }

        /* ---- 运行 ---- */
        case STATE_RUNNING:
        {
            unsigned char ramp_max;
            char delta;

            /* ① 充电线插入：立即停机 */
            if (VBUS_SENSE == 0)
            {
                motor_stop();
                throttle_index = 0;
                state = STATE_CHARGING;
                break;
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
                    led_b_on();  delay_ms(250);
                    led_all_off(); delay_ms(250);
                }
                state = STATE_SLEEP;
                break;
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

                /* 一阶低通滤波平滑 PWM */
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

                /* 堵转超时：停机回到 0 档 */
                if (++motor_stuck_time >= MOTOR_STUCK_TIMEOUT)
                {
                    motor_stop();
                    throttle_index = 0;
                }
            }
            else if (B_start)
            {
                /* 注意：motor_step() 依赖 B_start==1 压制比较器中断，
                 * 必须在 motor_start() 完成后才能清零。 */
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
                break;
            }

            led_show_throttle();
            break;
        }
        } /* end switch */
    }
}

/* ======== 按键事件处理 ======== */
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
            break;   /* 休眠/充电时忽略短按 */

        if (bat_is_low())
        {
            bat_low_beebee();
            break;
        }
        if (++throttle_index >= THROTTLE_SUM)
        {
            throttle_index = 0;
            motor_stop();
        }
        if (throttle_index == 1)
            B_start = 1;
        break;

    case KEY_EVT_LONG:
        if (state == STATE_SLEEP || state == STATE_CHARGING)
        {
            /* 有充电线时不允许开机 */
            if (VBUS_SENSE == 0)
            {
                state = STATE_CHARGING;
                break;
            }
            POWER_SWITCH = 1;
            throttle_index = 0;
            pwm_value  = 0;
            last_pwm   = 0;
            motor_time = 0;
            state = STATE_RUNNING;
            led_all_off();
            led_r_on();   /* 红灯：0 档待机 */
        }
        else /* STATE_RUNNING */
        {
            motor_stop();
            throttle_index = 0;
            state = STATE_SLEEP;
        }
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

/* ======== LED 档位显示 ======== */
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

/* ======== 初始化 ======== */
void init(void)
{
    clk_init();
    led_init();
    key_init();
    motor_init();
    timer2_init();
    bat_init();
    P_SW2 |= 0x80;   /* 保持 xdata SFR 访问使能（PWMA 寄存器需要）*/
    EA = 1;
    /* POWER_SWITCH 在首次进入 STATE_SLEEP 时由状态机关闭，
     * 不在此处提前关闭，以确保上电后 LED 双闪正常可见。 */
}

/* ======== 时钟初始化（35MHz 内部 RC）======== */
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

/* ======== Timer2：10ms 节拍（35MHz，12T）======== */
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
    WDT_CONTR |= 0x10;   /* 喂狗 */
    tick_10ms = 1;
    key_scan();
}
