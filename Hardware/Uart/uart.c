#include "uart.h"
#include "system.h"
#include <stdio.h>
#include <string.h>

u8  TX1_Cnt;            //发送计数
u8  RX1_Cnt;            //接收计数
bit B_TX1_Busy;         //发送忙标志
u16 Rx1_Timer  = 0;
u8  revFlag = 0;

u8  TX3_Cnt;            //发送计数
u8  RX3_Cnt;            //接收计数
bit B_TX3_Busy;         //发送忙标志
u16 Rx3_Timer  = 0;


u8  RX1_Buffer[MAX_LENGTH]; //接收缓冲
u8  RX3_Buffer[MAX_LENGTH]; //接收缓冲

//========================================================================
// 函数名称: void Uart1_Init(void)
// 函数功能: UART0初始化,9600位波特率/秒,8位字符,1位停止位,无校验.
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 波特率9600,时钟源选择为ACLK.误码率可能会大些.
//           用户可选择其他高频时钟源.高频时钟会使波特率的误码率降低.
//========================================================================
void Uart1_Init(void)		//9600bps@6MHz
{  
    /*********** 波特率使用定时器1 *****************/
    {
        TR1 = 0;
        AUXR &= ~0x01;      //S1 BRT Use Timer1;
        AUXR |=  (1<<6);    //Timer1 set as 1T mode
        TMOD &= ~(1<<6);    //Timer1 set As Timer
        TMOD &= ~0x30;      //Timer1_16bitAutoReload;
//        TH1 = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate3) / 256);      //_11MHz
//        TL1 = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate3) % 256);

        TL1 = 0x64;			//设置定时初始值_6MHz
	    TH1 = 0xFF;			//设置定时初始值_6MHz
        
        ET1 = 0;    //禁止中断
        INTCLKO &= ~0x02;  //不输出时钟
        TR1  = 1;
    }

    /*************************************************/
    //UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 
    //           0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率 

    SCON = (SCON & 0x3f) | 0x40; 


   
    ES  = 1;            //允许中断
    REN = 1;
    PS  = 0;            //中断高优先级
    PSH = 1;
    
    //UART1 switch to, 0x00: P3.0 P3.1, 0x40: P3.6 P3.7, 
    //                 0x80: P1.6 P1.7, 0xC0: P4.3 P4.4
    P_SW1 &= 0x3f;
    P_SW1 |= 0x40;  
    
    B_TX1_Busy = 0;
    TX1_Cnt = 0;
    RX1_Cnt = 0;
}


//重定向Printf
char putchar(char c )
{
    Uart485_EN(1);
    S3BUF = c;
    while(!S3TI);
    S3TI = 0;
    Uart485_EN(0);
    return c;
}

void UART1_ISR (void) interrupt 4
{
    if(RI)
    {
        if(SysRunState.isSleep == 1)
    	{
      		SysRunState.isSleep = 0;
    		SysRunState.NoUartTime = 0;
            BLUE_WAKEUP();
    	}
    
        RI = 0;
        revFlag = 1;
        Rx1_Timer = 0;
        RX1_Buffer[RX1_Cnt] = SBUF;
        if(++RX1_Cnt >= MAX_LENGTH)   
        {
            RX1_Cnt = 0;
        }
    }

    if(TI)
    {
        TI = 0;
        B_TX1_Busy = 0;
    }
}


//========================================================================
// 函数名称: void uart_send(u8 *buf, u8 len)
// 函数功能: 串口发送函数
// 入口参数: @*buf：发送的数据；@len：数据长度
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void Uart3_Init(void)		//9600bps@6MHz
{
    T2R = 0;
	S3CON = 0x10;		//8位数据,可变波特率
	S3CON &= 0xBF;		//串口3选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器时钟1T模式
//	T2L = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate3)% 256);			//设置定时初始值_11MHz
//	T2H = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate3)/ 256);		//设置定时初始值
 	T2L = 0x64;			//设置定时初始值——6M
    T2H = 0xFF;			//设置定时初始值
	ET2 = 0;			//禁止定时器中断
	AUXR |= 0x10;		//定时器2开始计时
	
    ES3  = 1;            //允许中断
    PS3  = 0;            //中断高优先级
    PS3H = 0;

    S3REN = 1;            //允许接收
    
    P_SW2 &= ~0x02; 
    
    B_TX3_Busy = 0;
    TX3_Cnt = 0;
    Rx3_Timer  = 0;
    RX3_Cnt = 0;
}

void UART3_ISR (void) interrupt 17
{
    if(S3RI)
    {
        S3RI = 0;
        Rx3_Timer = 0;
        RX3_Buffer[RX3_Cnt] = S3BUF;
        if(++RX3_Cnt >= MAX_LENGTH)   
        {
            RX3_Cnt = 0;
        }
    }

    if(S3TI)
    {
        S3TI = 0;
        B_TX3_Busy = 0;
    }
}

void uartble_send(u8 *buf,  u8 len)
{
    u8 i;
    for (i=0;i<len;i++)     
    {
        SBUF = buf[i];
        B_TX1_Busy = 1;
        while(B_TX1_Busy);
    }
}


void uart485_send(u8 *buf, u8 len)
{
    u8 i;
    Uart485_EN(1);
    for (i=0;i<len;i++)     
    {
        S3BUF = buf[i];
        B_TX3_Busy = 1;
        while(B_TX3_Busy);
    }
    Uart485_EN(0);
}

void Uart3Hnd()
{
    if (Rx3_Timer > 20)
    //if(RX3_Cnt > 1)
    {
        Rx3_Timer = 0;
        uart485_send(RX3_Buffer,RX3_Cnt);
        ClearRs485Buf();
    }
}



void ClearRs485Buf()
{
    memset(RX3_Buffer,0,MAX_LENGTH);
    RX3_Cnt = 0;
}

void ClearBleBuf()
{
    memset(RX1_Buffer,0,MAX_LENGTH);
    RX1_Cnt = 0;
}


