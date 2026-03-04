#ifndef __KEY_H__
#define __KEY_H__

#include "STC8Hxxx.h"

sbit KEY1 = P3^7;

#define KEY_SHORTPRESSED    0x01
#define KEY_LONGPRESSED     0x02
#define KEY_DOUBLEPRESSED   0x04

#define KEY_LONGPRESS_TH    100  // 100 * 10ms = 1s
#define KEY_DOUBLEPRESS_TH  15   // 15 * 10ms = 150ms

#define THROTTLE_SUM        4

#define STA_SLEEP           0x01
#define STA_WAKING          0x02
#define STA_RUNNING         0x04
#define STA_BATLOW          0x08
#define STA_CHARGING        0x10

void KeyInit(void);
void KeyScan(void);
void KeyHandle(void);

#endif
