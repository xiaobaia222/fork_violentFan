#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "STC8Hxxx.h"

#define POWER_SWITCH        P31

/* 油门档位表 */
#define THROTTLE_SUM        4
extern unsigned char code THROTTLE[THROTTLE_SUM];

/* 电机参数 */
#define PWM_START_VALUE     40      // 开环启动目标油门 (20~250)
#define MOTOR_STUCK_TIMEOUT 80      // 堵转超时 80*10ms = 800ms

/* 运行状态，供 main.c 读取 */
extern bit B_run;
extern unsigned char motor_stuck_time;

void motor_init(void);
void motor_start(void);
void motor_stop(void);
void motor_enter_run(void);    /* 开环启动后切入闭环 */
void motor_set_pwm(unsigned char val);

#endif
