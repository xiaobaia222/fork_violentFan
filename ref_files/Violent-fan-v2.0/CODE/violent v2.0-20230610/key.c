#include "key.h"

extern bit B_start;
extern unsigned char throttle_index;
extern unsigned int motor_time;
extern unsigned char status;

extern void MotorStop(void);
extern void BatLowBeeBee(void);

static unsigned char data key_stage = 0;
static unsigned char data key_pressed_time = 0;
static unsigned char data Trg = 0;

void KeyInit(void)
{
    key_stage = 0;
    key_pressed_time = 0;
    Trg = 0;
}

void KeyScan(void)
{
    switch(key_stage)
    {
        case 0:
        {
            if(KEY1 == 0)
                key_stage = 1;
            break;
        }
        case 1:
        {
            if(KEY1 == 0)
            {
                if(++key_pressed_time == KEY_LONGPRESS_TH)
                {
                    Trg = KEY_LONGPRESSED;
                    key_stage = 3;
                }
            }
            else
            {
                key_stage = 2;
                key_pressed_time = 0;
            }
            break;
        }
        case 2:
        {
            if(KEY1 == 1)
            {
                if(++key_pressed_time == KEY_DOUBLEPRESS_TH)
                {
                    Trg = KEY_SHORTPRESSED;
                    key_stage = 3;
                }
            }
            else
            {
                Trg = KEY_DOUBLEPRESSED;
                key_stage = 3;
            }
            break;
        }
        case 3:
        {
            if(KEY1 == 1)
            {
                key_stage = 0;
                key_pressed_time = 0;
            }
            break;
        }
        default:
            break;
    }
}

void KeyHandle(void)
{
    if(Trg == 0)
        return;
    motor_time = 0;
    switch(Trg)
    {
        case KEY_SHORTPRESSED:
        {
            if(status & STA_BATLOW)
            {
                BatLowBeeBee();
            }
            else
            {
                if(++throttle_index == 1)
                    B_start = 1;
                if(throttle_index == THROTTLE_SUM)
                {
                    throttle_index = 0;
                    MotorStop();
                }
            }
            break;
        }
        case KEY_LONGPRESSED:
        {
            MotorStop();
            status &= ~STA_RUNNING;
            status |= STA_SLEEP;
            break;
        }
        case KEY_DOUBLEPRESSED:
        {
            MotorStop();
            break;
        }
        default:
            break;
    }
    Trg = 0;
}
