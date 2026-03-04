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
static bit B_timer1_of = 0;

static unsigned char data pwm_val = 0;
static unsigned char data step = 0;
static unsigned char data demag_step = 0;   // 0:已消磁 1:待消磁 2:正在消磁
unsigned char data motor_stuck_time = 0;

static unsigned int data phase_time = 0;
static unsigned int data phase_buf[8];
static unsigned char data time_idx = 0;

/* ======== 内部函数声明 ======== */
static void gpio_init(void);
static void pwm_init(void);
static void adc_init(void);
static void cmp_init(void);
static void timer0_init(void);
static void timer1_init(void);
static void timer0_reload(unsigned int n);
static unsigned int timer1_get_cnt(void);
static void motor_step(void);

/* ======== GPIO 初始化 ======== */
static void gpio_init(void)
{
    /* P1.0~P1.5 推挽输出，用于 PWM 驱动 */
    P1M0 |= 0x3F;
    P1M1 &= ~0x3F;
    P1 &= ~0x3F;

    /* P3.2~P3.6 高阻输入，用于 ADC/比较器 */
    P3M0 &= ~0x7C;
    P3M1 |= 0x7C;

    POWER_SWITCH = 1;
}

/* ======== PWM 初始化 (PWMA 3通道 + 死区) ======== */
static void pwm_init(void)
{
    P_SW2 |= 0x80;

    PWM_AH = 0; PWM_AL = 0;
    PWM_BH = 0; PWM_BL = 0;
    PWM_CH = 0; PWM_CL = 0;

    PWMA_PSCR  = 3;        // 预分频 Fck/(3+1), PWM频率 = 35M/4/256 ≈ 34kHz
    PWMA_DTR   = 24;       // 死区 24 个定时器时钟
    PWMA_ARR   = 255;      // 自动重装值，8位分辨率
    PWMA_CCER1 = 0;
    PWMA_CCER2 = 0;
    PWMA_SR1   = 0;
    PWMA_SR2   = 0;
    PWMA_ENO   = 0;
    PWMA_PS    = 0;
    PWMA_IER   = 0;

    /* 通道1 (A相): PWM模式1, 预装载允许 */
    PWMA_CCMR1  = 0x68;
    PWMA_CCR1   = 0;
    PWMA_CCER1 |= 0x01;

    /* 通道2 (B相) */
    PWMA_CCMR2  = 0x68;
    PWMA_CCR2   = 0;
    PWMA_CCER1 |= 0x10;

    /* 通道3 (C相) */
    PWMA_CCMR3  = 0x68;
    PWMA_CCR3   = 0;
    PWMA_CCER2 |= 0x01;

    PWMA_BKR = 0x80;       // 主输出使能
    PWMA_CR1 = 0x81;       // 使能计数器, 自动重装缓冲, 边沿对齐, 向上计数
    PWMA_EGR = 0x01;       // 产生一次更新事件
}

/* ======== ADC 初始化 (为比较器提供正输入) ======== */
static void adc_init(void)
{
    ADC_CONTR = 0x80 + 13; // ADC 上电 + 通道13
    ADCCFG = 0x21;
    P_SW2 |= 0x80;
    ADCTIM = 0x20 + 20;
}

/* ======== 比较器初始化 (P3.6 为反相输入, ADC引脚为正输入) ======== */
static void cmp_init(void)
{
    CMPCR1 = 0x8C;         // 比较器使能, P3.6 反相输入, ADC引脚正输入
    CMPCR2 = 60;           // 60个时钟滤波
}

/* ======== Timer0: 换相/消磁定时 (12T, 模式0) ======== */
static void timer0_init(void)
{
    AUXR &= 0x7F;          // 12T 模式
    TMOD &= 0xF0;          // 16位自动重装
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

/* ======== Timer1: 测量换相间隔 (12T, 模式0) ======== */
static void timer1_init(void)
{
    AUXR &= 0xBF;          // 12T 模式
    TMOD &= 0x0F;          // 16位自动重装
    TL1 = 0;
    TH1 = 0;
    TF1 = 0;
    ET1 = 1;
    TR1 = 1;
}

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

/* ======== 六步换相 ========
 *  step  高端PWM  低端ON  检测相(ADC)  比较器沿
 *   0    A(ch1)   BL      C(ch13)      下降
 *   1    A(ch1)   CL      B(ch12)      上升
 *   2    B(ch2)   CL      A(ch11)      下降
 *   3    B(ch2)   AL      C(ch13)      上升
 *   4    C(ch3)   AL      B(ch12)      下降
 *   5    C(ch3)   BL      A(ch11)      上升
 */
static void motor_step(void)
{
    switch (step)
    {
    case 0: /* AB */
        PWMA_ENO = 0x00; PWM_AL = 0; PWM_CL = 0;
        delay_500ns();
        PWMA_ENO = 0x01;
        PWM_BL = 1;
        ADC_CONTR = 0x80 + 13;
        CMPCR1 = 0x8C + 0x10;      // 下降沿中断
        break;
    case 1: /* AC */
        PWMA_ENO = 0x01; PWM_AL = 0; PWM_BL = 0;
        delay_500ns();
        PWM_CL = 1;
        ADC_CONTR = 0x80 + 12;
        CMPCR1 = 0x8C + 0x20;      // 上升沿中断
        break;
    case 2: /* BC */
        PWMA_ENO = 0x00; PWM_AL = 0; PWM_BL = 0;
        delay_500ns();
        PWMA_ENO = 0x04;
        PWM_CL = 1;
        ADC_CONTR = 0x80 + 11;
        CMPCR1 = 0x8C + 0x10;
        break;
    case 3: /* BA */
        PWMA_ENO = 0x04; PWM_BL = 0; PWM_CL = 0;
        delay_500ns();
        PWM_AL = 1;
        ADC_CONTR = 0x80 + 13;
        CMPCR1 = 0x8C + 0x20;
        break;
    case 4: /* CA */
        PWMA_ENO = 0x00; PWM_BL = 0; PWM_CL = 0;
        delay_500ns();
        PWMA_ENO = 0x10;
        PWM_AL = 1;
        ADC_CONTR = 0x80 + 12;
        CMPCR1 = 0x8C + 0x10;
        break;
    case 5: /* CB */
        PWMA_ENO = 0x10; PWM_AL = 0; PWM_CL = 0;
        delay_500ns();
        PWM_BL = 1;
        ADC_CONTR = 0x80 + 11;
        CMPCR1 = 0x8C + 0x20;
        break;
    }

    if (B_start)
        CMPCR1 = 0x8C;             // 开环启动期间禁止比较器中断
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
    pwm_val = 0;
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
    unsigned char i;
    for (i = 0; i < 8; i++)
        phase_buf[i] = 0;
    demag_step = 0;
    CMPCR1 &= ~0x40;
    if (step & 0x01)
        CMPCR1 = 0xAC;     // 上升沿中断
    else
        CMPCR1 = 0x9C;     // 下降沿中断
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

/* Timer0: 换相延时 + 消磁延时 */
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
        timer0_reload(phase_time / 4);
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

/* 比较器: 反电动势过零检测 */
void CMP_Isr(void) interrupt 21
{
    unsigned char data i;

    CMPCR1 &= ~0x40;               // 清中断标志

    if (demag_step == 0)
    {
        phase_buf[time_idx] = timer1_get_cnt();
        if (B_timer1_of)
        {
            B_timer1_of = 0;
            phase_buf[time_idx] = 65535;
        }
        time_idx = (++time_idx) % 8;

        phase_time = 0;
        for (i = 0; i < 8; i++)
            phase_time += phase_buf[i];
        phase_time >>= 3;

        if (phase_time >= 40 && phase_time <= 8192)
            motor_stuck_time = 0;

        timer0_reload(phase_time / 4);
        demag_step = 1;
    }
}
