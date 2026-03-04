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

/* 换相提前角: 0=0deg, 1=7.5deg, 2=15deg, 3=22.5deg */
#define ADVANCE_LEVEL       2

/* 占空比变化率限制 (每 10ms tick) */
#define PWM_RAMP_SLOW       3       // 低速 (phase_time > 500)
#define PWM_RAMP_FAST       10      // 高速

/* 运行状态，供 main.c 读取/写入 */
extern bit B_run;
extern bit desync_flag;
extern unsigned char motor_stuck_time;
extern unsigned int phase_time;
extern unsigned int zero_crosses;

void motor_init(void);
void motor_start(void);
void motor_stop(void);
void motor_enter_run(void);
void motor_set_pwm(unsigned char val);
void motor_update_filter(void);     /* 10ms tick 中调用，更新自适应滤波等级 */

#endif
