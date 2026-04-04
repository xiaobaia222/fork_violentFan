#ifndef __ADC_BAT_H__
#define __ADC_BAT_H__

#include "STC8Hxxx.h"

/*
 * 电池电压检测 (ADC 通道 10)
 *
 * 硬件分压: 470 / (330 + 470)
 * 目标: 2S 锂电低压阈值 ~6.4V
 *
 * 说明:
 *  - motor.c 已经初始化 ADC (ADCCFG/ADCTIM 等)
 *  - 本模块只在电机未运行时采样, 避免影响 BEMF 比较器
 */

/* 10 位 ADC, Vref≈VCC, 6.4V≈3.76V → 3.76/5*1023≈770 */
#define BAT_ADC_LOW      770u

void        bat_init(void);
void        bat_update_10ms(void);       /* 在 10ms tick 中调用 */
bit         bat_is_low(void);            /* 是否低压标志 */
unsigned int bat_get_adc(void);          /* 返回滤波后的 ADC 值 */

#endif

