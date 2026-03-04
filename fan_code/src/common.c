#include "..\inc\common.h"
#include "intrins.h"


void delay_ms(unsigned short ms)
{
    unsigned char i, j;
	while(ms--)
	{
        _nop_();
        _nop_();
        i = 46;
        j = 113;
        do
        {
            while (--j);
        } while (--i);

	}
}

void delay_us(unsigned short us)
{
	unsigned char data i;
   do
   { 
	    _nop_( );
	    i = 6;
	    while (--i);
   }while(--us);
}

void delay_500ns(void)
{
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
}