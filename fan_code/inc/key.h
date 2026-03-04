#ifndef __KEY_H__
#define __KEY_H__

#include "STC8Hxxx.h"

#define KEY_PIN             P37

/* 时间参数，单位：扫描周期（10ms） */
#define KEY_DEBOUNCE_CNT    3       // 30ms 消抖
#define KEY_LONG_CNT        100     // 1s 长按
#define KEY_DOUBLE_GAP      25      // 250ms 双击窗口

/* 按键事件 */
#define KEY_EVT_NONE        0x00
#define KEY_EVT_SHORT       0x01
#define KEY_EVT_LONG        0x02
#define KEY_EVT_DOUBLE      0x04

void          key_init(void);
void          key_scan(void);           /* 10ms 定时器中断中调用 */
unsigned char key_get_event(void);      /* 主循环中调用，读后自动清除 */

#endif
