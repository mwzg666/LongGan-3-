#include "Sensor.h"
#include "mcp4725.h"
#include "system.h"
#include "main.h"
#include "CMD.h"
#include "uart.h"
#include "i2c.h"
#include "flash.h"
#include "CalcDoseRate.h"
#include "DoseRate.h"

LP_SYSTEM_STTAE SysRunState={0};
extern void Adc_Init();
extern uint16_t DeviceGetBatVal(void);
extern void DeviceGetBatAlarm(STU_BATTERY *bat);
extern STU_BATTERY s_Bat;

//unsigned int ADC16Result = 0; 

//========================================================================
// 函数名称: void InitParam()
// 函数功能: 初始化各个参数
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================

void InitParam()
{
	memset((void*)&SysRunState.stParam,0,sizeof(LP_PARAM));
	SysRunState.stParam.SensorType = SENSOR_LONGPOLE;          //探头类型
	SysRunState.stParam.s_SysParam.Hv = 800;                   //高压值
	SysRunState.stParam.s_SysParam.Z1 = 2500-200;              //甄别器阈值1
	SysRunState.stParam.s_SysParam.Ct = 1000;                  //计数时间
	SysRunState.stParam.s_SysParam.Hd = 3;                     //高压误差
	SysRunState.stParam.s_SysParam.Z2 = 1100;                  //甄别器阈值2
	SysRunState.stParam.s_SysParam.Hn = 0x5a;

	SysRunState.stParam.s_SysParam.DiYaCanshuA = 1.06581410364015E-14;//0.63;         //低量程通道校准因子
	SysRunState.stParam.s_SysParam.DiYaCanshuB = 0.291478787484166;      //0.00019;   //低量程通道校准因子
	SysRunState.stParam.s_SysParam.DiYaCanshuC = 0.0108551044292504;     //0.83;      //1;低量程通道校准因子
	SysRunState.stParam.s_SysParam.GaoYaCanshuA = 36242.9291771224;      //33.6;        //高量程通道校准因子
	SysRunState.stParam.s_SysParam.GaoYaCanshuB = 13.9188068943568;      //0.000023;    //高量程通道校准因子
	SysRunState.stParam.s_SysParam.GaoYaCanshuC = 0.000875735882434164;//0.83;        //高量程通道校准因子
	
	SysRunState.stParam.s_Alarm.DosePreAlarm = 300;            //300uSv
	SysRunState.stParam.s_Alarm.DoseAlarm = 400;               //400uSv
	SysRunState.stParam.s_Alarm.DoseRatePreAlarm = 300;        //300uSv/h
	SysRunState.stParam.s_Alarm.DoseRateAlarm = 400;           //400uSv/h

	WritePara();
}

void DevInit(void)
{
    BLUE_PWOFF();
    GDoseSeg = LOW_SEG;
	BLUE_CFG_MODE();
	BLUE_WAKEUP();
}
void sysSleep(void)
{
  	SysRunState.isSleep = 1;
}

void DevSleep(void)
{
	BLUE_CFG_MODE();
	delay_ms(2500);
	uartble_send("AT+CLEARADDR\r\n",14);    //清除远端蓝牙地址
	delay_ms(500);
	BLUE_WORK_MODE();
	BLUE_SLEEP();
	sysSleep();
	SensorMeasureBegin();//开始测量 
}

void Error()
{
    while(1)
    {
        //Light_M(1);
        delay_ms(50);
        //Light_M(0);
        delay_ms(50);
    }
}

void BtInit()
{
    BLUE_WORK_MODE();
    BLUE_SLEEP();
    BLUE_PWOFF();
    delay_ms(500);
    
    BLUE_PWON();
    BLUE_WAKEUP();
    BLUE_CFG_MODE(); 

    
	uartble_send("AT+ROLE=0\r\n",11);//从模式
	delay_ms(500);
	uartble_send("AT+LOWPOWER=1\r\n",15);//允许低功耗
	delay_ms(500);
	uartble_send("AT+TXPOWER=7\r\n",14);//发射功率0~7
	delay_ms(500);
    
	uartble_send("AT+CLEARADDR\r\n",14);//清除远端蓝牙地址
	delay_ms(500);
	uartble_send("AT+AUTH=1\r\n",11);//设置鉴权
	delay_ms(500);
	uartble_send("AT+BIND=1\r\n",11);//设置绑定地址
	delay_ms(500);
//    uartble_send("AT+LADDR?\r\n",11);
//    delay_ms(500);
//    uartble_send("AT+RADDR?\r\n",11);
//    delay_ms(500);
	/*uartble_send("AT+RADDR?\r\n",11);//
	delay_ms(500);
	uartble_send("AT+BLECONNPARAM?\r\n",18);
	delay_ms(1000);
	uartble_send("AT+DFU\r\n",8);
	delay_ms(1000);*/

    
    BLUE_WORK_MODE();
}


//========================================================================
// 函数名称: void delay_ms(WORD ms) _11MHz 
// 函数功能: 毫秒延时函数
// 入口参数: @WORD ms：延时多少毫秒
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
//void delay_ms(WORD ms)              
//{
//    WORD t = 1000;
//    while(ms--)
//    {
//        for (t=0;t<1000;t++) ;
//    }
//}


void delay_ms(WORD ms)	//@6.000MHz
{
	DWORD i;
    while(ms--)
    {
    	_nop_();
    	_nop_();
    	_nop_();
        i = 1498UL;
    	while (i) i--;
    }
}


void delay_us(BYTE us)
{
    while(us--)
    {
        ;
    }
}

//========================================================================
// 函数名称: void IoInit()
// 函数功能: 单片机I/O口初始化
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void IoInit()
{
    EAXFR = 1;
    WTST = 0;                       //设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快

    P0M1 = 0xFC;   P0M0 = 0x00;     //设置为准双向口           P0M1 = 0x50
    P1M1 = 0xFB;   P1M0 = 0x00;     //设置为准双向口           P1M1 = 0x02    
    P2M1 = 0xCF;   P2M0 = 0x00;     //设置为准双向口
    P3M1 = 0x0C;   P3M0 = 0x00;     //P3.3设置为推挽输出
    P4M1 = 0xB1;   P4M0 |=(1<<1)|(1<<2)|(1<<3)|(1<<6);     //设置为准双向口
    P5M1 = 0x0B;   P5M0 |=(1<<2);     //设置为准双向口
}

//========================================================================
// 函数名称: void TimerTask()
// 函数功能: 定时任务，通过定时器0定时10ms来设置相关任务
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void TimerTask()
{
    u16 delta = 0;
    static u16 Time1s = 0;
    if(Timer0Cnt)
    {
        delta = Timer0Cnt * 10;
        Timer0Cnt = 0;
        if(RX1_Cnt>0)
        {
            Rx1_Timer += delta;
        }
        if(RX3_Cnt>0)
        {
            Rx3_Timer += delta;
        }
        Time1s += delta;
        if(Time1s >= 1000)                      //100*10=1000ms
        {         
            Time1s = 0;
            SysRunState.isCanReadSensor = 1;
        }
        Light_Run(delta);
        
    }
}

//========================================================================
// 函数名称: void BleHnd()
// 函数功能: 通过BLE与上位机握手
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void BleHnd()
{
    if(revFlag)
    {
        if(Rx1_Timer > 20)                  //串口超时20ms
        {
            Rx1_Timer = 0;
            DataPro(RX1_Buffer,RX1_Cnt);
            //uart485_send(RX1_Buffer,RX1_Cnt);
            ClearBleBuf();  
            revFlag = 0;
        }
    }
    else
    {
        if(SysRunState.NoUartTime > POWER_OFF_TIME)
        {
            DevSleep();
        }
        else
        {
            sysSleep();
        }
    }
}

void Light_Run(u16 dt)
{
    static u16 counter = 0;
    u16 compare = 5000;
    counter += dt;
    if(counter > compare)
    {
        counter = 0;
        Light_M(0);
    }
    else if(counter >= (compare - 100))
    {      
        Light_M(1);
    }
}

int main(void)
{    
    SysInit();
    IoInit();
    
    Light_M(1);
    Adc_Init();
	DevInit();
    delay_ms(200);
    
    Uart1_Init();
    ClearBleBuf();
    //Uart3_Init();
    //ClearRs485Buf();
         
    delay_ms(500);
    
    Timer0_Init();
    
	Timer3_Init();
	Timer4_Init();
    
    delay_ms(500);
    
	GetPara(&SysRunState.stParam);
    delay_ms(1000);
    
    SensorInit();
    
    Light_M(0);
  
	EA = 1;
    //ADC_CONTR &= ~(1<<8);     //不使能 ADC 模块
//    IDL = 1;
//    _nop_();
//    _nop_();
//    _nop_();
//    _nop_();
	//开机先检测一次电池电量
    DeviceGetBatVal();

    BtInit();
    DeviceGetBatVal();
    revFlag = 0;
	BLUE_SLEEP();
	DeviceGetBatAlarm(&s_Bat);//开机先检测一次电池电量

	SensorMeasureBegin();//开始测量 
	InitArr();
    MCP4725_OutVol(MCP4725_S1_ADDR,2500-(WORD)SysRunState.stParam.s_SysParam.Z1);
	delay_ms(100);
    while(1)
    {   
        TimerTask();           
        if(SysRunState.isCanReadSensor == 1)
        {
           
            CaptureSensorPluseCounter(); //捕获当前测量结果
            SensorMeasureBegin();         //开始测量 
            SysRunState.isCanReadSensor = 0;
        }

        if((SysRunState.NoUartTime >= 90*100)&&(SysRunState.NoUartTime < 93*100))
        {
            //长时间无通信数据尝试复位蓝牙模块
            BtInit();
            RX1_Cnt = 0;
            revFlag = 0;
            SensorMeasureBegin();//开始测量 
            SysRunState.NoUartTime = 93*100;
        }
        BleHnd();
        //Uart3Hnd();
        
    }
}




