#include "..\inc\adc_bat.h"
#include "..\inc\motor.h"
#include "intrins.h"

extern bit B_start;

/* 环形缓冲: 最多 16 点, 去极值后平均 */
static unsigned int data bat_buf[16];
static unsigned char data bat_idx   = 0;
static unsigned char data bat_count = 0;

static unsigned int data bat_adc   = 0;
static bit               bat_low  = 0;

/* 读取指定 ADC 通道 (0~15), 返回 10 位结果 */
static unsigned int adc_read_channel(unsigned char ch)
{
    unsigned int val;
    unsigned char tmp;

    /* 保留高 4 位 (PWR/START/FLAG), 只改通道 */
    tmp = ADC_CONTR & 0xF0;
    ADC_CONTR = tmp | (ch & 0x0F) | 0x80;   /* 使能 ADC 电源 */
    _nop_(); _nop_();

    ADC_CONTR |= 0x40;                      /* 启动转换 */
    while (!(ADC_CONTR & 0x20));            /* 等待完成 */
    ADC_CONTR &= ~0x20;                     /* 清完成标志 */

    val = ((unsigned int)ADC_RES << 8) | ADC_RESL;
    return val;
}

void bat_init(void)
{
    unsigned char i;
    bat_idx   = 0;
    bat_count = 0;
    bat_adc   = 0;
    bat_low   = 0;
    for (i = 0; i < sizeof(bat_buf) / sizeof(bat_buf[0]); i++)
        bat_buf[i] = 0;
}

/* 10ms 调用一次, 在主循环中执行 */
void bat_update_10ms(void)
{
    unsigned int val;
    unsigned int sum;
    unsigned int maxv, minv;
    unsigned char i;
    unsigned char n;

    /* 电机启动/运行期间不切 ADC 通道, 避免影响 BEMF 比较器 */
    if (B_run || B_start)
        return;

    /* 每 10ms 采样 1 点, 慢速滚动更新 */
    val = adc_read_channel(10);     /* 通道 10: 电池电压 */
    bat_buf[bat_idx] = val;
    bat_idx++;
    if (bat_idx >= sizeof(bat_buf) / sizeof(bat_buf[0]))
        bat_idx = 0;

    if (bat_count < sizeof(bat_buf) / sizeof(bat_buf[0]))
        bat_count++;

    if (bat_count < 4)
        return;                     /* 样本太少, 暂不判定 */

    /* 去极值平均 */
    maxv = bat_buf[0];
    minv = bat_buf[0];
    sum  = 0;
    for (i = 0; i < bat_count; i++)
    {
        unsigned int v = bat_buf[i];
        sum += v;
        if (v > maxv) maxv = v;
        if (v < minv) minv = v;
    }

    if (bat_count > 2)
    {
        sum -= maxv;
        sum -= minv;
        n = bat_count - 2;
    }
    else
    {
        n = bat_count;
    }

    if (n == 0)
        return;

    bat_adc = sum / n;
    bat_low = (bat_adc < BAT_ADC_LOW);
}

bit bat_is_low(void)
{
    return bat_low;
}

unsigned int bat_get_adc(void)
{
    return bat_adc;
}

