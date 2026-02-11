#ifndef __LED_H__
#define __LED_H__

#include "STC8Hxxx.h"

#define	LED_R	P17
#define	LED_G	P54
#define	LED_B	P16

void led_init(void);
void led_r_on(void);
void led_g_on(void);
void led_b_on(void);
void led_r_off(void);
void led_g_off(void);
void led_b_off(void);
void led_all_off(void);
void led_all_on(void);

#endif