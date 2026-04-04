#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "..\inc\key.h"
#include "..\inc\motor.h"
#include "..\inc\adc_bat.h"
#include "intrins.h"

/* ======== 全局状态 ======== */
bit B_start = 0;
unsigned char throttle_index = 0;
unsigned int motor_time = 0;
unsigned char status = STA_SLEEP;   /* 上电默认休眠，由长按唤醒 */
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

    /* 上电绿灯闪 2 次，表明 MCU 正常启动，然后进入休眠等待长按 */
    led_g_on();  delay_ms(150);
    led_g_off(); delay_ms(100);
    led_g_on();  delay_ms(150);
    led_g_off();

    while (1)
    {
        key_handle();

        if (!tick_10ms)
            continue;
        tick_10ms = 0;

        /* ========= 唤醒/充电状态处理 ========= */
        if (status & STA_WAKING)
        {
            /* 退出唤醒标志，一次性处理 */
            status &= ~STA_WAKING;

            if (VBUS_SENSE == 0)        /* 插着充电线，进入充电休眠 */
            {
                motor_stop();
                throttle_index = 0;
                POWER_SWITCH = 0;
                status |= STA_CHARGING;
                status |= STA_SLEEP;
            }
            else                        /* 未在充电，正常唤醒进入运行 */
            {
                POWER_SWITCH = 1;
                status &= ~STA_SLEEP;
                status &= ~STA_CHARGING;
                status |= STA_RUNNING;
                led_all_off();
                led_r_on();             /* 红灯亮起表示唤醒成功，等待短按切档 */
            }
        }

        /* 运行过程中插入充电线：模拟 INT4 中断行为，立即停机并休眠充电 */
        if ((status & STA_RUNNING) && (VBUS_SENSE == 0))
        {
            motor_stop();
            throttle_index = 0;
            POWER_SWITCH = 0;
            status &= ~STA_RUNNING;
            status |= STA_CHARGING;
            status |= STA_SLEEP;
        }

        /* ========= 电池电压检测 + 低压标志 ========= */
        bat_update_10ms();
        if (bat_is_low())
            status |= STA_BATLOW;
        else
            status &= ~STA_BATLOW;

        /* 低压运行保护: 停机, 蓝灯闪烁提示 ~2 秒, 然后休眠 */
        if ((status & STA_BATLOW) && (status & STA_RUNNING))
        {
            unsigned char blink;
            motor_stop();
            throttle_index = 0;
            bat_low_beebee();
            for (blink = 0; blink < 4; blink++)
            {
                led_all_off();
                led_b_on();
                delay_ms(250);
                led_all_off();
                delay_ms(250);
            }
            status &= ~STA_RUNNING;
            status |= STA_SLEEP;
        }

        /* ========= 休眠状态：关闭驱动并进入空闲低功耗 ========= */
        if (status & STA_SLEEP)
        {
            if (B_run || B_start)
            {
                motor_stop();
                B_start = 0;
            }
            throttle_index = 0;
            led_all_off();
            POWER_SWITCH = 0;

            /* MCU 进入 IDL 模式，任意中断唤醒 (Timer2/按键) */
            PCON |= 0x01;
            continue;
        }

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
            /* 注意: 不能在 motor_start() 之前清零 B_start。
             * motor_step() 依赖 B_start==1 来压制比较器中断，
             * 确保开环强制换相阶段不被 BEMF ISR 干扰。 */
            motor_start();
            B_start = 0;    /* 开环结束后才清零，允许闭环比较器检测 */
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
        /* 休眠/唤醒状态下的短按：不做任何操作，避免误触 */
        if (status & (STA_SLEEP | STA_WAKING))
            break;

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
        if (status & STA_SLEEP)
        {
            /* 休眠时长按：进入唤醒流程，在主循环中根据 VBUS_SENSE 决定充电或运行 */
            status |= STA_WAKING;
        }
        else
        {
            /* 运行/充电时长按：关机休眠 */
            motor_stop();
            throttle_index = 0;
            status &= ~STA_RUNNING;
            status |= STA_SLEEP;
        }
        break;

    case KEY_EVT_DOUBLE:
        if (status & STA_SLEEP)
        {
            /* 休眠时双击忽略 */
            break;
        }
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
        case 0: led_r_on(); break;      /* 待机: 红灯常亮 */
        case 1: led_g_on(); break;      /* 1档: 绿 */
        case 2: led_b_on(); break;      /* 2档: 蓝 */
        case 3: led_r_on(); led_g_on(); led_b_on(); break;  /* 3档: 全亮白 */
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
    bat_init();
    P_SW2 |= 0x80;     /* 保持 xdata SFR 访问使能，运行时 PWMA 寄存器需要 */
    EA = 1;

    /* 上电默认休眠，立即关闭 MOS 驱动电源 */
    if (status & STA_SLEEP)
        POWER_SWITCH = 0;
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
    WDT_CONTR |= 0x10;     /* 喂狗，防止看门狗复位 */
    tick_10ms = 1;
    key_scan();
}
