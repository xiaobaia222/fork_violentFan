#include "..\inc\led.h"

void led_init(void)
{
    P1M0 |= 0xC0;
    P1M1 |= 0xC0;
    P5M0 |= 0x10;
    P5M1 |= 0x10;
}

void led_r_on(void)
{
    LED_R = 0;
}

void led_g_on(void)
{
    LED_G = 0;
}

void led_b_on(void)
{
    LED_B = 0;
}

void led_r_off(void)
{
    LED_R = 1;
}

void led_g_off(void)
{
    LED_G = 1;
}

void led_b_off(void)
{
    LED_B = 1;
}

void led_all_off(void)
{
    led_r_off();
    led_g_off();
    led_b_off();
}

void led_all_on(void)
{
    led_r_on();
    led_g_on();
    led_b_on();
}
