#include "STC8Hxxx.h"
#include "intrins.h"
//#include "stdio.h"

sbit PWM1   = P1^0;
sbit PWM1_L = P1^1;
sbit PWM2   = P1^2;
sbit PWM2_L = P1^3;
sbit PWM3   = P1^4;
sbit PWM3_L = P1^5;
/*************************************************************************/
//三色LED，共阳极
#define	LED_R	P17
#define	LED_G	P54
#define	LED_B	P16
/*************************************************************************/
#define	VBUS_SENSE		P30		//充电线接入检测，0：充电线接入，1：充电线断开
#define	POWER_SWITCH	P31		//mos管驱动器电源开关，0：断开，1：接通
/*************************************************************************/
#define	KEY1					P37
#define	KEY_SHORTPRESSED	0x01
#define	KEY_LONGPRESSED		0x02
#define	KEY_DOUBLEPRESSED	0x04
#define	KEY_LONGPRESS_TH		100			//按键长按阈值= 100 * 10ms = 1s
#define	KEY_DOUBLEPRESS_TH	15			//按键双击阈值= 15 * 10ms = 150ms（两次单击按键的时间小于150ms才认为是双击）
unsigned char data key_stage = 0;
unsigned char data key_pressed_time = 0;
unsigned char data Trg = 0;
/*************************************************************************/
#define	BAT_ADC_CH		10		//电压检测使用ADC通道10
#define	BAT_AVG_SUM		16		//16个数求一次平均
#define	BAT_LOW_TH		6.4		//低压保护电压值：6.4v
#define	V_DIV_SCALE		(470.0 / (330.0 + 470.0))		//电池电压分压比
#define	ADC_VREF			5.0		//ADC参考电压：5.0v
#define	CODE_TO_V_SCALE	(ADC_VREF / 1024.0) / V_DIV_SCALE
#define	BATLOW_RECOVERY_TIME	100
unsigned int bat_code[BAT_AVG_SUM];
unsigned char bat_sample_cnt = 0;
float bat_v;		//电池电压，单位：v
unsigned int batlow_recovery_time = 0;
unsigned int batlow_cnt = 0;
/*************************************************************************/
#define	THROTTLE_SUM	4
unsigned char throttle_index = 0;
unsigned char code THROTTLE[4] = {0, 60, 150, 250};
/*************************************************************************/
#define	BEEBEE_VOLUME				40	//提示音音量(0~250)
#define	PWM_START_VALUE			40	//启动油门(20~250)
#define	MOTOR_STUCK_TIMEOUT	80  //电机堵转50*10ms=500ms后，关闭电调
#define	MOTOR_RUNTIME_TH		30000		//电机工作时间阈值30000*10ms = 5min，在这个时间之内没有任何按键操作将自动关机
#define	MOTOR_IDLETIME_TH		3000		//电机闲置时间阈值3000*10ms = 30sec，	在这个时间之内没有任何按键操作将自动关机
unsigned char	PWM_value;	//决定PWM占空比的值
bit	B_start = 0;					//启动模式
bit	B_RUN = 0;						//运行标志
bit	B_Timer1_OF = 0;
unsigned int data PhaseTimeTmp[8];			//8个换相时间, 其sum/16就是30度电角度
unsigned char data step = 0;						//换相步骤
unsigned char data demag_step = 0;			//0:已经消磁, 1:需要消磁, 2:正在消磁,
unsigned char data motor_stuck_time = 0;		//堵转超时变量
unsigned int motor_time = 0;						//电机工作时间
/*************************************************************************/
bit	tick_10ms;		//10ms定时标志

#define	STA_SLEEP		0x01
#define	STA_WAKING	0x02
#define	STA_RUNNING	0x04
#define	STA_BATLOW	0x08
#define	STA_CHARGING	0x10
unsigned char status = STA_WAKING;		//上电后默认状态是低功耗休眠

void Delay1ms(unsigned char n)		//@35.00MHz
{
	unsigned char i, j;
	while(n--)
	{
		i = 35;
		j = 8;
		do
		{
			while (--j);
		} while (--i);
	}
}

void Delay1us(unsigned char us)		//@35.00MHz
{
	unsigned char i;
	do
	{
		_nop_();
		_nop_();
		i = 9;
		while (--i);
	}
	while(--us);
}
void Delay500ns(void)
{
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
}
unsigned int	ADC_Get10bitResult(unsigned char channel)	//channel = 0~15
{
	unsigned char i;
	ADC_CONTR = 0xC0 | channel; 
	//Delay500ns();
	//while((ADC_CONTR & ADC_FLAG) == 0)	;	//等待ADC结束
	i = 255;
	while(i--)
	{
		if((ADC_CONTR & 0x20) != 0)
			break;	//等待ADC结束
	}
	ADC_CONTR &= ~0x20;
	return	(((unsigned int)ADC_RES << 8) | ADC_RESL);
}
unsigned int bat_average(void)	//去掉最大最小值后求平均
{
	unsigned char i;
	unsigned int min, max;
	unsigned int sum = 0;
	min = max = bat_code[0];
	for(i = 0; i < BAT_AVG_SUM; i++)
	{
		sum += bat_code[i];
		if(bat_code[i] >= max)
			max = bat_code[i];
		if(bat_code[i] <= min)
			min = bat_code[i];
	}
	sum -= (max + min);
	sum /= (BAT_AVG_SUM - 2);
	return sum;
}
void MotorStep(void) 	// 换相序列函数
{
	switch(step)
	{
		case 0:  // AB  PWM1, PWM2_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM3_L=0;
			Delay500ns();
			PWMA_ENO = 0x01;			// 打开A相的高端PWM
			PWM2_L = 1;						// 打开B相的低端
			ADC_CONTR = 0x80+13;	// 选择P0.2作为ADC输入 即C相电压
			CMPCR1 = 0x8c + 0x10;	// 比较器下降沿中断
			break;
		case 1:  // AC  PWM1, PWM3_L=1
			PWMA_ENO = 0x01;	PWM1_L=0;	PWM2_L=0;	// 打开A相的高端PWM
			Delay500ns();
			PWM3_L = 1;						// 打开C相的低端
			ADC_CONTR = 0x80+12;	// 选择P0.1作为ADC输入 即B相电压
			CMPCR1 = 0x8c + 0x20;	// 比较器上升沿中断
			break;
		case 2:  // BC  PWM2, PWM3_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM2_L=0;
			Delay500ns();
			PWMA_ENO = 0x04;			// 打开B相的高端PWM
			PWM3_L = 1;						// 打开C相的低端
			ADC_CONTR = 0x80+11;	// 选择P0.0作为ADC输入 即A相电压
			CMPCR1 = 0x8c + 0x10;	// 比较器下降沿中断
			break;
		case 3:  // BA  PWM2, PWM1_L=1
			PWMA_ENO = 0x04;	PWM2_L=0;	PWM3_L=0;	// 打开B相的高端PWM
			Delay500ns();
			PWM1_L = 1;						// 打开C相的低端
			ADC_CONTR = 0x80+13;	// 选择P0.2作为ADC输入 即C相电压
			CMPCR1 = 0x8c + 0x20;	// 比较器上升沿中断
			break;
		case 4:  // CA  PWM3, PWM1_L=1
			PWMA_ENO = 0x00;	PWM2_L=0;	PWM3_L=0;
			Delay500ns();
			PWMA_ENO = 0x10;			// 打开C相的高端PWM
			PWM1_L = 1;						// 打开A相的低端
			bat_code[bat_sample_cnt] = ADC_Get10bitResult(BAT_ADC_CH);
			ADC_CONTR = 0x80+12;		// 选择P0.1作为ADC输入 即B相电压
			CMPCR1 = 0x8c + 0x10;	//比较器下降沿中断
			break;
		case 5:  // CB  PWM3, PWM2_L=1
			PWMA_ENO = 0x10;	PWM1_L=0;	PWM3_L=0;	// 打开C相的高端PWM
			Delay500ns();
			PWM2_L = 1;						// 打开B相的低端
			ADC_CONTR = 0x80+11;	// 选择P0.0作为ADC输入 即A相电压
			CMPCR1 = 0x8c + 0x20;	// 比较器上升沿中断
			break;
		default:
			break;
	}
	if(B_start)
		CMPCR1 = 0x8C;	// 启动时禁止下降沿和上升沿中断               
}                                                                                                                                                                       
void MotorStart(void) // 强制电机启动函数
{
	unsigned int timer,i;
	CMPCR1 = 0x8C;	// 关比较器中断
	PWM_value  = 10;	// 初始占空比, 根据电机特性设置
	PWMA_CCR1L = PWM_value;
	PWMA_CCR2L = PWM_value;
	PWMA_CCR3L = PWM_value;
	step = 0;
	timer = 200;		//风扇电机启动
	MotorStep();
	Delay1ms(100);	//Delay1ms(250);// 初始位置	
	while(1)
	{
		for(i=0; i<timer; i++)//根据电机加速特性, 最高转速等等调整启动加速速度
		{
			Delay1us(100);
		}
		++step;
		step %= 6;
		PWM_value += 2;
		if(PWM_value > PWM_START_VALUE)
			PWM_value = PWM_START_VALUE;
		PWMA_CCR1L = PWM_value;
		PWMA_CCR2L = PWM_value;
		PWMA_CCR3L = PWM_value;	 
		MotorStep();
		timer -= timer/15+1;		
		if(timer < 25)
			return;	
	}
}
void MotorStop(void)
{
	B_RUN = 0;
	throttle_index = 0;
	PWM_value = 0;
	CMPCR1 = 0x8C;	// 关比较器中断
	PWMA_ENO  = 0;	// 关闭PWM输出
	PWMA_CCR1L = 0;
	PWMA_CCR2L = 0;
	PWMA_CCR3L = 0;
	PWM1_L=0;
	PWM2_L=0;
	PWM3_L=0;
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
void GPIOInit(void)
{
	//P17:LED-R, P16:LED-B, P15:LIN3, P14:HIN3
	//P13:LIN2, P12:HIN2, P11:LIN1, P10:HIN1
	P1M0 = 0xFF;	
	P1M1 = 0x00;
	P1 = 0xC0;
	//P37:KEY1, P36:CMP-, P35:ADC13, P34:ADC12
	//P33:ADC11, P32:ADC10, P31:POWER_SWITCH, P30:VBUS_SENSE
	P3M0 = 0x00;	
	P3M1 = 0x7C;
	P3 = 0x81;
	//P54:LED-G
	P5M0 = 0x10;	
	P5M1 = 0x00;
	P5 = 0x10;
}
void ADCInit(void)	//ADC初始化函数(为了使用ADC输入端做比较器信号, 实际没有启动ADC转换)
{
	ADC_CONTR = 0x80 + 13;	//ADC PowerOn + channel
	ADCCFG = 0x21;
	P_SW2 |=  0x80;	//访问XSFR
	ADCTIM = 0x20 + 20;
}
void PWMInit(void)
{
	P_SW2 |= 0x80;		//SFR enable   

	PWM1   = 0;
	PWM1_L = 0;
	PWM2   = 0;
	PWM2_L = 0;
	PWM3   = 0;
	PWM3_L = 0;

	PWMA_PSCR = 3;		// 预分频寄存器, 分频 Fck_cnt = Fck_psc/(PSCR[15:0}+1), 边沿对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)), 中央对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_DTR  = 24;		// 死区时间配置, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,  
						//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARR    = 255;	// 自动重装载寄存器,  控制PWM周期
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
	PWMA_ENO    = 0;
	PWMA_PS     = 0;
	PWMA_IER    = 0;
//	PWMA_ISR_En = 0;

	PWMA_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR1   = 0;			// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= 0;		// 选择IO, 0:选择P1.0 P1.1, 1:选择P2.0 P2.1, 2:选择P6.0 P6.1, 
//	PWMA_ENO   |= 0x01;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_CCMR2  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR2   = 0;			// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER1 |= 0x50;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<2);	// 选择IO, 0:选择P1.2 P1.3, 1:选择P2.2 P2.3, 2:选择P6.2 P6.3, 
//	PWMA_ENO   |= 0x04;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_CCMR3  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMA_CCR3   = 0;			// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCER2 |= 0x05;		// 开启比较输出, 高电平有效
	PWMA_PS    |= (0<<4);	// 选择IO, 0:选择P1.4 P1.5, 1:选择P2.4 P2.5, 2:选择P6.4 P6.5, 
//	PWMA_ENO   |= 0x10;		// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x81;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐模式, 向上计数,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		// 产生一次更新事件, 清除计数器和与分频计数器, 装载预分频寄存器的值
}
void CMPInit(void)	//比较器初始化程序
{
	CMPCR1 = 0x8C;	// 1000 1100 打开比较器，P3.6作为比较器的反相输入端，ADC引脚作为正输入端 
	CMPCR2 = 60;		// 60个时钟滤波，比较结果变化延时周期数, 0~63
}
void Timer0Init(void)	//Timer0初始化函数，用于检测到过零后什么时候换相以及换相后什么时候开始检测过零
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x00;		//设置定时初始值
	TH0 = 0x00;		//设置定时初始值
	TF0 = 0;			//清除TF0标志
	ET0 = 1; 			// 允许ET0中断
	//TR0 = 1; // 打开定时器0
}
void Timer1Init(void)	//timer1初始化函数,用于计算换相之间的时间
{
	AUXR &= 0xBF;		//定时器时钟12T模式
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0x00;		//设置定时初始值
	TH1 = 0x00;		//设置定时初始值
	TF1 = 0;		//清除TF1标志
	ET1 = 1;		//开溢出中断
	TR1 = 1;		//定时器1开始计时
}
void Timer2Init(void)		//10毫秒@35.00MHz，产生10MS时基信号
{
	AUXR &= 0xFB;		//定时器时钟12T模式
	T2L = 0x11;			//设置定时初始值
	T2H = 0x8E;			//设置定时初始值
	IE2 |= 0x04;    //使能定时器中断
	AUXR |= 0x10;		//定时器2开始计时
}
void TM0_Isr(void) interrupt 1	//timer0中断函数
{
	TR0 = 0;	// Timer0停止运行
	if(demag_step == 1)		//标记需要消磁. 每次检测到过0事件后第一次中断为30度角延时, 设置消磁延时.
	{
		demag_step = 2;		//1:需要消磁, 2:正在消磁, 0已经消磁
		if(B_RUN)	//电机正在运行
		{
			if(++step >= 6)
				step = 0;
			MotorStep();
		}
		//消磁时间, 换相后线圈(电感)电流减小到0的过程中, 出现反电动势, 电流越大消磁时间越长, 过0检测要在这个时间之后
		//100%占空比时施加较重负载, 电机电流上升, 可以示波器看消磁时间.
		//实际上, 只要在换相后延时几十us才检测过零, 就可以了
		TH0 = (unsigned char)((65536UL - 40*2) >> 8);	//装载消磁延时
		TL0 = (unsigned char)(65536UL - 40*2);
		TR0 = 1;	//Timer0开始运行
	}
	else if(demag_step == 2)
	{
		demag_step = 0;		//1:需要消磁, 2:正在消磁, 0已经消磁
	}
}
void TM1_Isr(void) interrupt 3	//timer1中断函数
{
	B_Timer1_OF = 1;	//溢出标志
}

void TM2_Isr(void) interrupt 12	//Timer2中断函数
{
	tick_10ms = 1;	//10ms定时标志
	KeyScan();
}
void CMP_Isr(void) interrupt 21	//比较器中断函数, 检测到反电动势过0事件
{
	static unsigned char data TimeIndex = 0;		// 换相时间保存索引
	unsigned int data PhaseTime;			// 换相时间计数
	unsigned char data i;
	CMPCR1 &= ~0x40;		// 比较器中断需软件清除中断标志位
	if(demag_step == 0)	// 消磁后才检测过0事件,   demag_step=1:需要消磁, =2:正在消磁, =0已经消磁
	{
		TR1 = 0;				//Timer1停止运行
		if(B_Timer1_OF)	//切换时间间隔(Timer1)有溢出
		{
			B_Timer1_OF = 0;
			PhaseTime = 8000;	//换相时间最大8ms, 2212电机12V空转最高速130us切换一相(200RPS 12000RPM), 480mA
		}
		else
		{
			PhaseTime = (((unsigned int)TH1 << 8) + TL1) >> 1;	//单位为1us
			if(PhaseTime >= 8000)	//换相时间最大8ms, 2212电机12V空转最高速130us切换一相(200RPS 12000RPM), 480mA
				PhaseTime = 8000;	
		}
		TH1 = 0;	
		TL1 = 0;
		TR1 = 1;	//Timer1开始运行

		PhaseTimeTmp[TimeIndex] = PhaseTime;	//保存一次换相时间
		TimeIndex = (++TimeIndex)%8;		//累加8次
		for(PhaseTime=0, i=0; i<8; i++)	//求8次换相时间累加和
			PhaseTime += PhaseTimeTmp[i];	
		PhaseTime = PhaseTime >> 4;			//求8次换相时间的平均值的一半, 即30度电角度
		if((PhaseTime >= 40) && (PhaseTime <= 1000))
			motor_stuck_time = 0;
		if(PhaseTime >= 80)
			PhaseTime -= 40;							//修正由于滤波电容引起的滞后时间
		else
			PhaseTime  = 20;
		
		// PhaseTime = 20;	//只给20us, 则无滞后修正, 用于检测滤波电容引起的滞后时间
		TR0 = 0;		//Timer0停止运行
		PhaseTime  = PhaseTime << 1;	//2个计数1us
		PhaseTime = 0 - PhaseTime;
		TH0 = (unsigned char)(PhaseTime >> 8);		//装载30度角延时
		TL0 = (unsigned char)PhaseTime;
		TR0 = 1;					//Timer0开始运行
		demag_step = 1;		//1:需要消磁, 2:正在消磁, 0已经消磁
		//下一步，在定时器0中断函数中进行换相
	}
}
void INT3_Isr() interrupt 11 //按键中断
{
}
void INT4_Isr() interrupt 16 //VBUS接入中断
{
	//IAP_CONTR = 0x60;
	status |= STA_CHARGING;
	status |= STA_SLEEP;
	status &= ~STA_RUNNING;
}
void SelfCheckBeeBee(void)
{
	PWMA_PSCR = 16;
	
	PWMA_CCR1L = BEEBEE_VOLUME;
	PWM2_L = 1;
	PWMA_ENO = 0x01;
	Delay1ms(250);
	PWMA_CCR1L = 0;
	PWM2_L = 0;
	Delay1ms(50);
	
	PWMA_PSCR = 14;
	
	PWMA_CCR2L = BEEBEE_VOLUME;
	PWM3_L = 1;
	PWMA_ENO = 0x04;
	Delay1ms(250);
	PWMA_CCR2L = 0;
	PWM3_L = 0;
	Delay1ms(50);
	
	PWMA_PSCR = 12;
	
	PWMA_CCR3L = BEEBEE_VOLUME;
	PWM1_L = 1;
	PWMA_ENO = 0x10;
	Delay1ms(250);
	PWMA_CCR3L = 0;
	PWM1_L = 0;
	Delay1ms(50);
	
	PWMA_ENO = 0x00;
	PWMA_PSCR = 3;
}
void BatLowBeeBee()
{
	PWMA_PSCR = 16;
	
	PWMA_CCR1L = BEEBEE_VOLUME;
	PWM2_L = 1;
	PWMA_ENO = 0x01;
	Delay1ms(100);
	PWMA_ENO = 0x00;
	Delay1ms(100);
	PWMA_ENO = 0x01;
	Delay1ms(100);
	PWMA_ENO = 0x00;
	PWMA_CCR1L = 0;
	PWM2_L = 0;
	
	PWMA_PSCR = 3;
}
void KeyHandle(void)
{
	if(Trg == 0)
		return;
	motor_time = 0;
	switch(Trg)
	{
		case KEY_SHORTPRESSED:		//短按切换挡位
		{
			if(status & STA_BATLOW)	//电池电压低时，禁止启动风扇
			{
				BatLowBeeBee();		//播放电池电压低的提示音
			}
			else
			{
				if(++throttle_index == 1)
					B_start = 1;
				if(throttle_index == 4)
				{
					throttle_index = 0;
					MotorStop();
				}
			}
			break;
		}
		case KEY_LONGPRESSED:			//长按按键关机
		{
			MotorStop();
			status &= ~STA_RUNNING;
			status |= STA_SLEEP;
			break;
		}
		case KEY_DOUBLEPRESSED:		//双击按键停止电机
		{
			MotorStop();
			break;
		}
		default:
			break;
	}
	Trg = 0;
}
void LEDControl(void)
{
	static unsigned int LED_cnt = 0;
	if(++LED_cnt == 100)
		LED_cnt = 0;
	if(status & STA_BATLOW)
	{
		LED_R = 1;
		LED_G = 1;
		LED_B = 0;
	}
	else if(status & STA_RUNNING)
	{
		if(throttle_index == 0)
		{
			LED_R = 0;
			LED_G = 1;
			LED_B = 1;
		}
		else
		{
			if(LED_cnt < (10 + throttle_index * 20))
			{
				LED_R = (LED_cnt / 10) % 2;
			}
			LED_G = 1;
			LED_B = 1;
		}
	}
}
void main(void)
{
	unsigned char i;
	GPIOInit();
	PWMInit();
	ADCInit();
	CMPInit();
	//UartInit();
	Timer0Init();	// Timer0初始化函数
	Timer1Init();	// Timer1初始化函数
	Timer2Init();	// Timer4初始化函数
	INTCLKO |= 0x20;  //使能INT3下降沿中断
	EA  = 1; 					//打开总中断
	while (1)
	{
		if(status & STA_RUNNING)	//运行
		{
			if(tick_10ms)		//10ms时隙
			{
				tick_10ms = 0;
				if(B_RUN)	//正在运行中
				{
					PWM_value = (unsigned char)(PWM_value * 0.9 + THROTTLE[throttle_index] * 0.1);
					PWMA_CCR1L = PWM_value;
					PWMA_CCR2L = PWM_value;
					PWMA_CCR3L = PWM_value;
					if(++motor_stuck_time == MOTOR_STUCK_TIMEOUT)	//堵转超时
						MotorStop();
				}
				else
				{	
					if(B_start)
					{
						B_start = 0;
						for(i=0; i<8; i++)
							PhaseTimeTmp[i] = 400;
						MotorStart();		// 启动电机
						B_start = 0;
						demag_step = 0;		// 初始进入时
						CMPCR1 &= ~0x40;	// 清除中断标志位
						if(step & 0x01)
							CMPCR1 = 0xAC;		//上升沿中断
						else
							CMPCR1 = 0x9C;		//下降沿中断
						B_RUN = 1;
						Delay1ms(250);	//延时一下, 先启动起来
						Delay1ms(250);
						motor_stuck_time = 0;		//启动超时时间 125*4 = 500ms
					}
					bat_code[bat_sample_cnt] = ADC_Get10bitResult(BAT_ADC_CH);
				}
				if(++motor_time == (throttle_index==0?MOTOR_IDLETIME_TH:MOTOR_RUNTIME_TH))
				{
					motor_time = 0;
					MotorStop();
					status &= ~STA_RUNNING;
					status |= STA_SLEEP;
				}
				if(++bat_sample_cnt == BAT_AVG_SUM)
				{
					bat_sample_cnt = 0;
					bat_v = bat_average() * CODE_TO_V_SCALE;	//计算得当前电压值
					//printf("batv:%.2f\n", bat_v + 0.005);
					if((status & STA_BATLOW) == 0)
					{
						if(bat_v < BAT_LOW_TH)
						{
							if(++batlow_cnt == 64)
							{
								batlow_cnt = 0;
								LED_B = 0;
								MotorStop();
								status |= STA_BATLOW;
								batlow_recovery_time = 0;
							}
						}
						else
							batlow_cnt = 0;
					}
					else 
					{
						if(bat_v > BAT_LOW_TH)
						{
							if(++batlow_cnt == 32)
							{
								batlow_cnt = 0;
								status &= ~STA_BATLOW;
							}
						}
						else
							batlow_cnt = 0;
					}
				}
				LEDControl();
			}
			KeyHandle();
		}
		if(status & STA_SLEEP)		//休眠
		{
			LED_R = 1;
			LED_G = 1;
			LED_B = 1;
			if(KEY1 == 0)			//如果按键还没松开，就延时等待松开
			{
				Delay1ms(200);
				Delay1ms(200);
				Delay1ms(200);
			}
			POWER_SWITCH = 0;	// 关闭MOS管驱动端电源
			INTCLKO &= ~0x40;	// 关闭INT4下降沿中断
			ADC_CONTR = 0x00;	// 关闭片内ADC电源
			CMPCR1 = 0;				// 关闭片内比较器
			IE2 &= ~0x04;    //使能定时器中断
			PCON = 0x02;			// MCU进入掉电模式
			//从这进入休眠，中断唤醒后执行完中断函数后也回到这
			_nop_();
			_nop_();
			IAP_CONTR = 0x60;
		}
		if(status & STA_WAKING)		//唤醒
		{
			unsigned char cnt = 0;
			while(KEY1 == 0)	
			{
				Delay1ms(200);
				LED_G = 0;
				Delay1ms(200);
				LED_G = 1;
				cnt++;
				if(cnt > 3)
					LED_R = 0;
			}
			if(cnt > 3)
			{
				status &= ~STA_SLEEP;
				status |= STA_WAKING;
				if(VBUS_SENSE == 0)		//如果正在充电（充电线接入）
				{
					POWER_SWITCH = 0;
					status |= STA_CHARGING;
					status |= STA_SLEEP;		//继续休眠
				}
				else									//未在充电							
				{		
					CMPCR1 = 0x8C;		//初始化比较器
					ADC_CONTR = 0x80;	//开ADC电源
					INTCLKO |= 0x40;	//使能INT4下降沿中断
					POWER_SWITCH = 1;	//开MOS管驱动端电源
					IE2 |= 0x04;    //使能定时器中断
					LED_R = 0;
					key_stage = 0;
					key_pressed_time = 0;
					Trg = 0;
					SelfCheckBeeBee();	  	//电机自检
					status |= STA_RUNNING;	//进入运行状态
				}
			}
			else
			{
				status &= ~STA_WAKING;
				status |= STA_SLEEP;
			}
			status &= ~STA_WAKING;//退出唤醒状态
		}
	}
}


