#include "..\inc\key.h"

/*
 * 标准嵌入式按键状态机，10ms 周期调用 key_scan()
 *
 *  IDLE ──按下──> PRESS_DEB ──消抖确认──> PRESSED（计时长按）
 *                   │                       │
 *                  弹跳                    松开
 *                   v                       v
 *                 IDLE                  WAIT_DOUBLE（等双击窗口）
 *                                        │        │
 *                                      再按下    窗口超时
 *                                        v        v
 *                                  DOUBLE_DEB   输出SHORT
 *                                      │
 *                                    消抖确认
 *                                      v
 *                                   输出DOUBLE -> WAIT_REL
 *
 *  PRESSED 中持续按住达到 KEY_LONG_CNT -> 输出LONG -> WAIT_REL
 *  WAIT_REL: 等待物理释放后回 IDLE
 */

#define ST_IDLE         0
#define ST_PRESS_DEB    1
#define ST_PRESSED      2
#define ST_WAIT_DOUBLE  3
#define ST_DOUBLE_DEB   4
#define ST_WAIT_REL     5

static unsigned char data state = ST_IDLE;
static unsigned short data cnt   = 0;
static unsigned char data event = KEY_EVT_NONE;

void key_init(void)
{
    P3M0 &= ~0x80;
    P3M1 &= ~0x80;
    P3 |= 0x80;

    P_SW2 |= 0x80;
    P3PU |= 0x80;   /* P3.7 按键上拉 */
    P3PU |= 0x01;   /* P3.0 VBUS_SENSE 上拉，防悬空误判充电接入 */
    P_SW2 &= ~0x80;

    state = ST_IDLE;
    cnt   = 0;
    event = KEY_EVT_NONE;
}

void key_scan(void)
{
    unsigned char pressed = (KEY_PIN == 0) ? 1 : 0;

    switch (state)
    {
    case ST_IDLE:
        if (pressed)
        {
            cnt = 0;
            state = ST_PRESS_DEB;
        }
        break;

    case ST_PRESS_DEB:
        if (pressed)
        {
            if (++cnt >= KEY_DEBOUNCE_CNT)
            {
                cnt = 0;
                state = ST_PRESSED;
            }
        }
        else
        {
            state = ST_IDLE;
        }
        break;

    case ST_PRESSED:
        if (pressed)
        {
            if (++cnt >= KEY_LONG_CNT)
            {
                event = KEY_EVT_LONG;
                state = ST_WAIT_REL;
            }
        }
        else
        {
            cnt = 0;
            state = ST_WAIT_DOUBLE;
        }
        break;

    case ST_WAIT_DOUBLE:
        if (pressed)
        {
            cnt = 0;
            state = ST_DOUBLE_DEB;
        }
        else
        {
            if (++cnt >= KEY_DOUBLE_GAP)
            {
                event = KEY_EVT_SHORT;
                state = ST_IDLE;
            }
        }
        break;

    case ST_DOUBLE_DEB:
        if (pressed)
        {
            if (++cnt >= KEY_DEBOUNCE_CNT)
            {
                event = KEY_EVT_DOUBLE;
                state = ST_WAIT_REL;
            }
        }
        else
        {
            state = ST_WAIT_DOUBLE;
        }
        break;

    case ST_WAIT_REL:
        if (!pressed){
            
            
            state = ST_IDLE;
        }
            
        break;

    default:
        state = ST_IDLE;
        break;
    }
}

unsigned char key_get_event(void)
{
    unsigned char e = event;
    event = KEY_EVT_NONE;
    return e;
}
