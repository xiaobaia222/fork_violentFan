#include "..\inc\motor.h"
#include "..\inc\common.h"
#include "intrins.h"

extern bit B_start;

/* ======== 引脚定义 ======== */
sbit PWM_AH = P1^0;
sbit PWM_AL = P1^1;
sbit PWM_BH = P1^2;
sbit PWM_BL = P1^3;
sbit PWM_CH = P1^4;
sbit PWM_CL = P1^5;

/* ======== 油门表 ======== */
unsigned char code THROTTLE[THROTTLE_SUM] = {0, 50, 80, 150};

/* ======== 运行变量 ======== */
bit B_run = 0;
bit desync_flag = 0;
static bit B_timer1_of = 0;

static unsigned char data pwm_val = 0;
static unsigned char data step = 0;
static unsigned char data demag_step = 0;
unsigned char data motor_stuck_time = 0;

/* IIR 滤波后的换相间隔 (替代原 phase_buf 8 点平均) */
unsigned int data phase_time = 0;
unsigned int zero_crosses = 0;

/* AM32 风格自适应滤波等级 */
static unsigned char data filter_level = 8;

/* 失同步检测 */
static unsigned int data last_phase_time = 0;

/* ======== 内部函数声明 ======== */
static void gpio_init(void);
static void pwm_init(void);
static void adc_init(void);
static void cmp_init(void);
static void timer0_init(void);
static void timer1_init(void);
static void timer0_reload(unsigned int n);
static unsigned int timer1_get_cnt(void);
static unsigned int timer1_read_cnt(void);
static void motor_step(void);

/* ======== GPIO 初始化 ======== */
static void gpio_init(void)
{
    P1M0 |= 0x3F;
    P1M1 &= ~0x3F;
    P1 &= ~0x3F;

    /* P3.0 VBUS_SENSE 高阻输入; P3.2~P3.6 ADC/比较器高阻 */
    P3M0 &= ~0x7D;
    P3M1 |= 0x7D;

    POWER_SWITCH = 1;
}

/* ======== PWM 初始化 (PWMA 3通道 + 死区) ======== */
static void pwm_init(void)
{
    P_SW2 |= 0x80;

    PWM_AH = 0; PWM_AL = 0;
    PWM_BH = 0; PWM_BL = 0;
    PWM_CH = 0; PWM_CL = 0;

    PWMA_PSCR  = 3;
    PWMA_DTR   = 24;
    PWMA_ARR   = 255;
    PWMA_CCER1 = 0;
    PWMA_CCER2 = 0;
    PWMA_SR1   = 0;
    PWMA_SR2   = 0;
    PWMA_ENO   = 0;
    PWMA_PS    = 0;
    PWMA_IER   = 0;

    PWMA_CCMR1  = 0x68;
    PWMA_CCR1   = 0;
    PWMA_CCER1 |= 0x01;

    PWMA_CCMR2  = 0x68;
    PWMA_CCR2   = 0;
    PWMA_CCER1 |= 0x10;

    PWMA_CCMR3  = 0x68;
    PWMA_CCR3   = 0;
    PWMA_CCER2 |= 0x01;

    PWMA_BKR = 0x80;
    PWMA_CR1 = 0x81;
    PWMA_EGR = 0x01;
}

/* ======== ADC 初始化 (为比较器提供正输入) ======== */
static void adc_init(void)
{
    ADC_CONTR = 0x80 + 13;
    ADCCFG = 0x21;
    P_SW2 |= 0x80;
    ADCTIM = 0x20 + 20;
}

/* ======== 比较器初始化 ======== */
static void cmp_init(void)
{
    CMPCR1 = 0x8C;
    CMPCR2 = 60;
}

/* ======== Timer0: 换相/消磁定时 ======== */
static void timer0_init(void)
{
    AUXR &= 0x7F;
    TMOD &= 0xF0;
    TL0 = 0;
    TH0 = 0;
    TF0 = 0;
    ET0 = 1;
}

static void timer0_reload(unsigned int n)
{
    n = 65535 - n;
    TR0 = 0;
    TH0 = n >> 8;
    TL0 = n;
    TR0 = 1;
}

/* ======== Timer1: 测量换相间隔 ======== */
static void timer1_init(void)
{
    AUXR &= 0xBF;
    TMOD &= 0x0F;
    TL1 = 0;
    TH1 = 0;
    TF1 = 0;
    ET1 = 1;
    TR1 = 1;
}

/* 破坏性读取：读后清零重启 */
static unsigned int timer1_get_cnt(void)
{
    unsigned int t;
    TR1 = 0;
    t = ((unsigned int)TH1 << 8) | TL1;
    TH1 = 0;
    TL1 = 0;
    TR1 = 1;
    return t;
}

/* 非破坏性读取：只读不清 (用于时间窗口检查) */
static unsigned int timer1_read_cnt(void)
{
    unsigned int t;
    t = ((unsigned int)TH1 << 8) | TL1;
    return t;
}

/* ======== 六步换相 ======== */
static void motor_step(void)
{
    switch (step)
    {
    case 0:
        PWMA_ENO = 0x00; PWM_AL = 0; PWM_CL = 0;
        delay_500ns();
        PWMA_ENO = 0x01;
        PWM_BL = 1;
        ADC_CONTR = 0x80 + 13;
        CMPCR1 = 0x8C + 0x10;
        break;
    case 1:
        PWMA_ENO = 0x01; PWM_AL = 0; PWM_BL = 0;
        delay_500ns();
        PWM_CL = 1;
        ADC_CONTR = 0x80 + 12;
        CMPCR1 = 0x8C + 0x20;
        break;
    case 2:
        PWMA_ENO = 0x00; PWM_AL = 0; PWM_BL = 0;
        delay_500ns();
        PWMA_ENO = 0x04;
        PWM_CL = 1;
        ADC_CONTR = 0x80 + 11;
        CMPCR1 = 0x8C + 0x10;
        break;
    case 3:
        PWMA_ENO = 0x04; PWM_BL = 0; PWM_CL = 0;
        delay_500ns();
        PWM_AL = 1;
        ADC_CONTR = 0x80 + 13;
        CMPCR1 = 0x8C + 0x20;
        break;
    case 4:
        PWMA_ENO = 0x00; PWM_BL = 0; PWM_CL = 0;
        delay_500ns();
        PWMA_ENO = 0x10;
        PWM_AL = 1;
        ADC_CONTR = 0x80 + 12;
        CMPCR1 = 0x8C + 0x10;
        break;
    case 5:
        PWMA_ENO = 0x10; PWM_AL = 0; PWM_CL = 0;
        delay_500ns();
        PWM_BL = 1;
        ADC_CONTR = 0x80 + 11;
        CMPCR1 = 0x8C + 0x20;
        break;
    }

    if (B_start)
        CMPCR1 = 0x8C;
}

/* ======== 开环强制启动 ======== */
void motor_start(void)
{
    unsigned int timer, i;

    CMPCR1 = 0x8C;
    pwm_val = 10;
    PWMA_CCR1L = pwm_val;
    PWMA_CCR2L = pwm_val;
    PWMA_CCR3L = pwm_val;

    step = 0;
    timer = 200;
    motor_step();
    delay_ms(100);

    while (1)
    {
        for (i = 0; i < timer; i++)
            delay_us(100);

        if (++step >= 6)
            step = 0;

        pwm_val += 2;
        if (pwm_val > PWM_START_VALUE)
            pwm_val = PWM_START_VALUE;

        PWMA_CCR1L = pwm_val;
        PWMA_CCR2L = pwm_val;
        PWMA_CCR3L = pwm_val;
        motor_step();

        timer -= timer / 15 + 1;
        if (timer < 25)
            return;
    }
}

/* ======== 停机 ======== */
void motor_stop(void)
{
    B_run = 0;
    desync_flag = 0;
    pwm_val = 0;
    zero_crosses = 0;
    phase_time = 0;
    last_phase_time = 0;
    CMPCR1 = 0x8C;
    PWMA_ENO   = 0;
    PWMA_CCR1L = 0;
    PWMA_CCR2L = 0;
    PWMA_CCR3L = 0;
    PWM_AL = 0;
    PWM_BL = 0;
    PWM_CL = 0;
}

/* ======== 开环启动后切入闭环 ======== */
void motor_enter_run(void)
{
    phase_time = 9000;
    last_phase_time = 9000;
    zero_crosses = 0;
    filter_level = 8;
    demag_step = 0;
    desync_flag = 0;
    CMPCR1 &= ~0x40;
    if (step & 0x01)
        CMPCR1 = 0xAC;
    else
        CMPCR1 = 0x9C;
    B_run = 1;
}

/* ======== 设置 PWM 占空比 ======== */
void motor_set_pwm(unsigned char val)
{
    pwm_val = val;
    PWMA_CCR1L = val;
    PWMA_CCR2L = val;
    PWMA_CCR3L = val;
}

/*
 * 自适应滤波等级更新 — 由 main.c 的 10ms tick 调用
 * 参考 AM32: 低速/启动高滤波, 高速低滤波
 */
void motor_update_filter(void)
{
    if (zero_crosses < 30 || phase_time > 2000)
        filter_level = 8;
    else if (phase_time > 500)
        filter_level = 4;
    else
        filter_level = 2;
}

/* ======== 总初始化 ======== */
void motor_init(void)
{
    gpio_init();
    pwm_init();
    adc_init();
    cmp_init();
    timer0_init();
    timer1_init();
}

/* ================================================================
 *  中断服务函数
 * ================================================================ */

/*
 * Timer0: 换相延时 + 消磁延时
 * demag_step 1 → 执行换相, 启动消磁定时 (phase_time/4)
 * demag_step 2 → 消磁结束, 允许下次过零检测
 */
void TM0_Isr(void) interrupt 1
{
    TR0 = 0;

    if (demag_step == 1)
    {
        demag_step = 2;
        if (B_run)
        {
            if (++step >= 6)
                step = 0;
            motor_step();
        }
        timer0_reload(phase_time >> 2);
    }
    else if (demag_step == 2)
    {
        demag_step = 0;
    }
}

/* Timer1: 溢出标记 */
void TM1_Isr(void) interrupt 3
{
    B_timer1_of = 1;
}

/*
 * 比较器 ISR — 反电动势过零检测 (AM32 风格改进)
 *
 * 三层防护:
 *  1. 时间窗口: Timer1 计数 < phase_time/2 则丢弃 (太早不可能是真 ZC)
 *  2. 连续采样: 读 CMPRES filter_level 次, 任一不一致则丢弃
 *  3. IIR 滤波: phase_time = 0.75*old + 0.25*new
 *
 * 失同步检测: 测量值与 phase_time 偏差 > 50% 则置 desync_flag
 */
void CMP_Isr(void) interrupt 21
{
    unsigned char data i;
    unsigned int data measured;
    unsigned int data half;
    unsigned int data diff;
    unsigned int data wait;
    unsigned char data expected;

    CMPCR1 &= ~0x40;

    if (demag_step != 0)
        return;

    /* --- 1. 时间窗口过滤 --- */
    if (phase_time > 0)
    {
        half = timer1_read_cnt();
        if (half < (phase_time >> 1))
            return;
    }

    /* --- 2. 连续采样验证 --- */
    expected = step & 0x01;
    for (i = 0; i < filter_level; i++)
    {
        if ((CMPCR1 & 0x01) != expected)
            return;
    }

    /* --- 读取并重置 Timer1 --- */
    measured = timer1_get_cnt();
    if (B_timer1_of)
    {
        B_timer1_of = 0;
        measured = 65535;
    }

    /* --- 3. IIR 滤波: new = 0.75*old + 0.25*measured --- */
    if (phase_time == 0)
        phase_time = measured;
    else
        phase_time = (phase_time * 3 + measured) >> 2;

    /* --- 4. 失同步检测: 偏差 > 50% --- */
    if (zero_crosses > 10 && last_phase_time > 0)
    {
        if (measured > phase_time)
            diff = measured - phase_time;
        else
            diff = phase_time - measured;

        if (diff > (phase_time >> 1))
            desync_flag = 1;
    }
    last_phase_time = phase_time;

    /* --- 堵转判定 --- */
    if (phase_time >= 40 && phase_time <= 8192)
        motor_stuck_time = 0;

    zero_crosses++;

    /* --- 换相延时: 30deg - advance --- */
    wait = (phase_time >> 1) - ((phase_time >> 3) * ADVANCE_LEVEL);
    if (wait < 10)
        wait = 10;
    timer0_reload(wait);
    demag_step = 1;
}
