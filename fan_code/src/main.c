#include "..\inc\main.h"
#include "..\inc\common.h"
#include "..\inc\led.h"
#include "intrins.h"
#include <stdio.h>



void init(void);
void clk_init(void);
void clk_out(void);
void main()
{
    init();
    
    
    while(1)
    {
        led_r_on();
        delay_ms(500);
        led_r_off();  
        delay_ms(500);
    }


}




void init(void)
{
    clk_init();
    led_init();
    // clk_out();
}

void clk_init(void)
{
     //选择35MHz
    P_SW2 = 0x80;
    CLKDIV = 0x04;
    IRTRIM = T35M_ROMADDR;
    VRTRIM = VRT35M_ROMADDR;
    IRCBAND = 0x01;
    CLKDIV = 0x00;
    
    P_SW2 = 0x00;


}

void clk_out()
{
    P5M0 = 0x00;
    P5M1 = 0x00;

    P_SW2 = 0x80;
    MCLKOCR = 0x01;                             //主时钟输出到P5.4口
//  MCLKOCR = 0x02;                             //主时钟2分频输出到P5.4口
    // MCLKOCR = 0x04;                             //主时钟4分频输出到P5.4口
//  MCLKOCR = 0x84;                             //主时钟4分频输出到P1.6口
    P_SW2 = 0x00;

}