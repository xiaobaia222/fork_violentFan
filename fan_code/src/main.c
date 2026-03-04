#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "..\inc\key.h"
#include "..\inc\motor.h"
#include "intrins.h"

bit B_start = 0;
unsigned char throttle_index = 0;
unsigned int motor_time = 0;
unsigned char status = 0;

void init(void);
void clk_init(void);
void timer2_init(void);
void key_handle(void);
void led_show_throttle(void);

void motor_stop(void)
{
    throttle_index = 0;
    B_start = 0;
}

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
        status &= ~STA_RUNNING;
        status |= STA_SLEEP;
        break;

    case KEY_EVT_DOUBLE:
        motor_stop();
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
    timer2_init();
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
    key_scan();
}
