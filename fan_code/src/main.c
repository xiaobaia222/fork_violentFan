#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "..\inc\key.h"
#include "..\inc\motor.h"
#include "intrins.h"

/* ======== 全局状态 ======== */
bit B_start = 0;
unsigned char throttle_index = 0;
unsigned int motor_time = 0;
unsigned char status = 0;
static bit tick_10ms = 0;
static unsigned char pwm_value = 0;

static unsigned char last_pwm = 0;

/* 自动关机阈值 */
#define MOTOR_RUNTIME_TH    30000   // 运行中 30000*10ms = 5min
#define MOTOR_IDLETIME_TH   3000    // 闲置时 3000*10ms = 30s

void init(void);
void clk_init(void);
void timer2_init(void);
void key_handle(void);
void led_show_throttle(void);

void bat_low_beebee(void)
{
}

void main()
{
    init();
    status = STA_RUNNING;

    while (1)
    {
        key_handle();

        if (!tick_10ms)
            continue;
        tick_10ms = 0;

        if (!(status & STA_RUNNING))
            continue;

        if (B_run)
        {
            motor_update_filter();

            /* 失同步恢复: 不停机, 油门减半重新建立同步 */
            if (desync_flag)
            {
                desync_flag = 0;
                pwm_value = pwm_value >> 1;
                zero_crosses = 0;
                motor_stuck_time = 0;
            }

            /* 一阶低通滤波平滑 PWM */
            pwm_value = (unsigned char)(
                ((unsigned int)pwm_value * 9 + THROTTLE[throttle_index]) / 10
            );

            /* 占空比变化率钳位 (AM32 防失步核心) */
            {
                unsigned char ramp_max;
                char delta;

                ramp_max = (phase_time > 500) ? PWM_RAMP_SLOW : PWM_RAMP_FAST;
                delta = (char)pwm_value - (char)last_pwm;
                if (delta > (char)ramp_max)
                    pwm_value = last_pwm + ramp_max;
                else if (delta < -(char)ramp_max)
                    pwm_value = last_pwm - ramp_max;
                last_pwm = pwm_value;
            }

            motor_set_pwm(pwm_value);

            if (++motor_stuck_time >= MOTOR_STUCK_TIMEOUT)
            {
                motor_stop();
                throttle_index = 0;
            }
        }
        else if (B_start)
        {
            B_start = 0;
            motor_start();
            B_start = 0;
            motor_enter_run();
            delay_ms(250);
            delay_ms(250);
            motor_stuck_time = 0;
            last_pwm = PWM_START_VALUE;
            pwm_value = PWM_START_VALUE;
        }

        /* 自动关机计时 */
        if (++motor_time >= (throttle_index == 0 ? MOTOR_IDLETIME_TH : MOTOR_RUNTIME_TH))
        {
            motor_time = 0;
            motor_stop();
            throttle_index = 0;
            status &= ~STA_RUNNING;
            status |= STA_SLEEP;
        }

        led_show_throttle();
    }
}

void key_handle(void)
{
    unsigned char evt = key_get_event();
    if (evt == KEY_EVT_NONE)
        return;

    motor_time = 0;

    switch (evt)
    {
    case KEY_EVT_SHORT:
        if (status & STA_BATLOW)
        {
            bat_low_beebee();
        }
        else
        {
            if (++throttle_index >= THROTTLE_SUM)
            {
                throttle_index = 0;
                motor_stop();
            }
            if (throttle_index == 1)
                B_start = 1;
        }
        break;

    case KEY_EVT_LONG:
        motor_stop();
        throttle_index = 0;
        status &= ~STA_RUNNING;
        status |= STA_SLEEP;
        break;

    case KEY_EVT_DOUBLE:
        motor_stop();
        throttle_index = 0;
        break;
    }
}

void led_show_throttle(void)
{
    static unsigned char last = 0xFF;
    if (throttle_index == last)
        return;
    last = throttle_index;

    led_all_off();
    switch (throttle_index)
    {
        case 1: led_r_on(); break;
        case 2: led_g_on(); break;
        case 3: led_b_on(); break;
        default: break;
    }
}

void init(void)
{
    clk_init();
    led_init();
    key_init();
    motor_init();
    timer2_init();
    P_SW2 |= 0x80;     /* 保持 xdata SFR 访问使能，运行时 PWMA 寄存器需要 */
    EA = 1;
}

void clk_init(void)
{
    P_SW2 = 0x80;
    CLKDIV = 0x04;
    IRTRIM = T35M_ROMADDR;
    VRTRIM = VRT35M_ROMADDR;
    IRCBAND = 0x01;
    CLKDIV = 0x00;
    P_SW2 = 0x00;
}

/* 10ms@35MHz, 12T 模式 */
void timer2_init(void)
{
    AUXR &= 0xFB;
    T2L = 0x11;
    T2H = 0x8E;
    IE2 |= 0x04;
    AUXR |= 0x10;
}

void TM2_Isr(void) interrupt 12
{
    tick_10ms = 1;
    key_scan();
}
