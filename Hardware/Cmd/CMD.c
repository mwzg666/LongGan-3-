#include <stdlib.h>
#include "CMD.h"
#include "sensor.h"
#include "mcp4725.h"
#include "flash.h"
#include "uart.h"
#include "mwpro.h"
#include "CalcCPS.h"
#include "Mcp4725.h"
#include "crc.h"

#define FRAM_DELAY delay_ms(100)

STU_CMD s_Head={0x68};
uint8_t snedbuf[100];

extern void DeviceGetBatAlarm(STU_BATTERY *bat);
extern void DevSleep(void);


//========================================================================
// 函数名称: WORD WordToSmall(WORD dat)
// 函数功能: 将WORD的数据转换为小端模式
// 入口参数: @WORD dat：要转换的数据
// 函数返回: 返回类型为WORD的小端模式数据
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================

WORD WordToSmall(WORD dat)
{
    BYTE buf[2];
    BYTE t;
    WORD ret;
    
    memcpy(buf, &dat, 2);
    t = buf[1];
    buf[1] = buf[0];
    buf[0] = t;
    
    memcpy(&ret, buf, 2);
    return ret;
}

float FloatToSmall(float dat)
{
    BYTE buf[4];
    BYTE t;
    float ret;
    
    memcpy(buf, &dat, 4);
    t = buf[3];
    buf[3] = buf[0];
    buf[0] = t;
    t = buf[2];
    buf[2] = buf[1];
    buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}

DWORD DwordToSmall(DWORD dat)
{
    BYTE buf[4];
    BYTE t;
    DWORD ret;
    
    memcpy(buf, &dat, 4);
    t = buf[3];
    buf[3] = buf[0];
    buf[0] = t;
    t = buf[2];
    buf[2] = buf[1];
    buf[1] = t;

    memcpy(&ret, buf, 4);
    return ret;
}


//========================================================================
// 函数名称: void GetPara(LP_PARAM *me)
// 函数功能: 从FLASH中读取参数，包括"控制参数"和"报警参数"
// 入口参数: @me：数据
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void GetPara(LP_PARAM *me)
{ 
      EEPROM_read(0,(u8 *)me,sizeof(LP_PARAM));
    if ( SysRunState.stParam.ParaCheckSum !=  CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2))
    {
          InitParam();
    }
    SysRunState.stParam.VerSion1 = VERSION1;
    SysRunState.stParam.VerSion2 = VERSION2;
    SysRunState.stParam.VerSion3 = VERSION3;
}

//========================================================================
// 函数名称: void WritePara()
// 函数功能: 写入数据到内存中
// 入口参数: @无
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
void WritePara()
{
    EA = 0;
    EEPROM_SectorErase(0);
    EEPROM_SectorErase(512);
    SysRunState.stParam.ParaCheckSum = CheckSum((BYTE *)&SysRunState.stParam,sizeof(LP_PARAM)-2);//add by kevin at 20150417
    if (!EEPROM_write(0, (u8 *)&SysRunState.stParam, sizeof(LP_PARAM)))
    {
        Error();
    }    
    EA = 1;
}


//向上位机发送命令
void SendData(uint8_t cmd, uint8_t *cdata, uint16_t length)
{
      uint16_t crc;
    //s_Head.head = 0x68;
    s_Head.cmd = cmd;
    s_Head.length = WordToSmall(length);
    memcpy(snedbuf,(uint8_t*)&s_Head,sizeof(STU_CMD));
    if(length>0)
    {
        memcpy(&snedbuf[sizeof(STU_CMD)],cdata,length);
    }
    crc = CRC16(snedbuf,length+sizeof(STU_CMD));
    crc = WordToSmall(crc);
    memcpy(&snedbuf[length+sizeof(STU_CMD)],(uint8_t*)&crc,2);
    snedbuf[length+sizeof(STU_CMD)+2] = 0x16;
    
    uartble_send(snedbuf,(u8)(length+7));
}

/*******************************************************************************
功能：获取上位机指定的模式
输入：无
输出：工作模式
*******************************************************************************/
int GetWorkMode(void)
{
  return 3;
}

void ReadFix()
{
    SendData(CMD_READ_FIX,(uint8_t*)&SysRunState.stParam.Fix,sizeof(float)*FIX_COUNT);
}

void WriteFix(BYTE *dat)
{
    memcpy((uint8_t*)&SysRunState.stParam.Fix,dat,sizeof(float)*FIX_COUNT);
    
    SendData(CMD_WRITE_FIX,NULL,0);
    SaveParam();
}


/*******************************************************************************
功能：命令分析与执行
输入：U32 id:指令ID号
      unsigned char dlc: 数据长度
      unsigned char *cdata:数据指针
输出：无
*******************************************************************************/
STU_CMD gs_CMD={0};
void DataPro(uint8_t *cdata, uint16_t length)
{  
      uint16_t i;
    uint16_t crcRev;
    uint16_t crcOut;
    //STU_CMD *pCmd = NULL;
    for(i=0;i<length; i++)
    {
          if(cdata[i] == 0x68)
        {
              //pCmd = (STU_CMD *)&cdata[i];
              memcpy(&gs_CMD,&cdata[i],sizeof(STU_CMD));
            gs_CMD.length = WordToSmall(gs_CMD.length);
            if((gs_CMD.length > length-7)||(cdata[i+6+gs_CMD.length] != 0x16))
            {
                  continue;
            }
            crcRev = cdata[i+4+gs_CMD.length] + cdata[i+5+gs_CMD.length]*256;
            crcOut = CRC16(&cdata[i],gs_CMD.length+4);
            if(crcRev != crcOut)
            {
                  continue;
            }
            SysRunState.NoUartTime = 0;

            switch(gs_CMD.cmd)
            {
                case 'C'://联络命令
                
                    ACK_CMD_C((uint8_t*)&SysRunState.stParam.SensorType);

                break;

                case 'V'://读计数
                    ACK_CMD_V();
                break;
                
                case 'E'://读工作状态
                    ACK_CMD_E();
                break;

                case 'R'://读参数
                    ACK_CMD_R();
                break;

                case 'W'://写参数
                 if(length-i-5 > sizeof(SYS_PRAM))
                 {
                    ACK_CMD_W(&cdata[i+4]);
                 }
                break;

                case 'S'://存参数
                    ACK_CMD_S();
                break;

                case 1://清除累计剂量
                    ACK_CMD_ClearDoseSum();
                break;
                
                case 2://清除最大剂量率
                    ACK_CMD_ClearMaxDoseRate();
                break;
                
                case 3://探测器自检
                    ACK_CMD_SelfCheck();
                break;
                
                case 4://开关探测器
                    ACK_CMD_SensorONOFF(cdata[i+4]);
                break;
                
                case 5://电池电量查询
                    ACK_CMD_Bat();
                break;

                case 6://报警确认
                    ACK_CMD_SureAlarm();
                break;

                case 7: // 量程切换
                    ACK_CMD_GmSw(cdata[i+4]);
                break;

                case 'B'://写报警参数
                    ACK_CMD_B(&cdata[i+4]);
                break;

                case 'F'://读报警参数
                    ACK_CMD_F();
                break;

                case CMD_READ_FIX:  ReadFix(); break;
                case CMD_WRITE_FIX: WriteFix(&cdata[i+4]);  break;
                
                case 0x28://远程升级
                      //asm(" mov &0xFFBE, PC;"); //跳转到升级代码
                  break;
            default:
              break;
            }
            i += (gs_CMD.length+4);
        }
    }
}

/*******************************************************************************
功能：联络命令响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_C(u8 *SensorType)
{
    SendData('C',SensorType,6);
}



/*******************************************************************************
功能：读命令(R)响应
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_R(void)
{
    SYS_PRAM red;
    red.Hv = FloatToSmall(SysRunState.stParam.s_SysParam.Hv);
    red.Ct = FloatToSmall(SysRunState.stParam.s_SysParam.Ct);
    red.Hd = FloatToSmall(SysRunState.stParam.s_SysParam.Hd);
    red.Hn = SysRunState.stParam.s_SysParam.Hn; 
    
    red.Z1 = FloatToSmall(SysRunState.stParam.s_SysParam.Z1);
    red.Z2 = FloatToSmall(SysRunState.stParam.s_SysParam.Z2);
    
    red.DiYaCanshuA = FloatToSmall(SysRunState.stParam.s_SysParam.DiYaCanshuA);
    red.DiYaCanshuB = FloatToSmall(SysRunState.stParam.s_SysParam.DiYaCanshuB);
    red.DiYaCanshuC = FloatToSmall(SysRunState.stParam.s_SysParam.DiYaCanshuC);
    
    red.GaoYaCanshuA = FloatToSmall(SysRunState.stParam.s_SysParam.GaoYaCanshuA);
    red.GaoYaCanshuB = FloatToSmall(SysRunState.stParam.s_SysParam.GaoYaCanshuB);
    red.GaoYaCanshuC = FloatToSmall(SysRunState.stParam.s_SysParam.GaoYaCanshuC);
   
    SendData('R',(uint8_t*)&red,sizeof(SYS_PRAM));
}


/*******************************************************************************
功能：写参数命令(W)响应
输入：unsigned char *cdata:参数
输出：无
*******************************************************************************/
void ACK_CMD_W(unsigned char *cdata)
{
    SYS_PRAM wcm;
    memcpy((uint8_t*)&wcm,cdata,sizeof(SYS_PRAM));
    SysRunState.stParam.s_SysParam.Hv = FloatToSmall(wcm.Hv);
    SysRunState.stParam.s_SysParam.Ct = FloatToSmall(wcm.Ct);
    SysRunState.stParam.s_SysParam.Hd = FloatToSmall(wcm.Hd);
    SysRunState.stParam.s_SysParam.Hn = wcm.Hn;
    
    SysRunState.stParam.s_SysParam.Z1 = FloatToSmall(wcm.Z1);
    SysRunState.stParam.s_SysParam.Z2 = FloatToSmall(wcm.Z2);

    SysRunState.stParam.s_SysParam.DiYaCanshuA = FloatToSmall(wcm.DiYaCanshuA);
    SysRunState.stParam.s_SysParam.DiYaCanshuB = FloatToSmall(wcm.DiYaCanshuB);
    SysRunState.stParam.s_SysParam.DiYaCanshuC = FloatToSmall(wcm.DiYaCanshuC);

    SysRunState.stParam.s_SysParam.GaoYaCanshuA = FloatToSmall(wcm.GaoYaCanshuA);
    SysRunState.stParam.s_SysParam.GaoYaCanshuB = FloatToSmall(wcm.GaoYaCanshuB);
    SysRunState.stParam.s_SysParam.GaoYaCanshuC = FloatToSmall(wcm.GaoYaCanshuC);

    SendData('W',NULL,0);
    //SaveParam();

}



/*******************************************************************************
功能：存参数命令(S)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/
void ACK_CMD_S(void)
{
    SendData('S',NULL,0);
    SaveParam();
    MCP4725_OutVol(MCP4725_S1_ADDR,2500-(WORD)SysRunState.stParam.s_SysParam.Z1);//alphy 阈值
}



/*******************************************************************************
功能：读计数命令(V)响应
输入：unsigned char SensorType:传感器类型
输出：无
*******************************************************************************/

void ACK_CMD_V(void)
{ 
    STU_DOSERATE gs_Dose;
    gs_Dose.C1 = FloatToSmall(SysRunState.s_DoseMSG.C1);
    gs_Dose.C2 = FloatToSmall(SysRunState.s_DoseMSG.C2);
    
    gs_Dose.Dose = FloatToSmall(SysRunState.s_DoseMSG.Dose);
    gs_Dose.DoseRate = FloatToSmall(SysRunState.s_DoseMSG.DoseRate);
    gs_Dose.DRSt = SysRunState.s_DoseMSG.DRSt;
    gs_Dose.MaxDoseRate = FloatToSmall(SysRunState.s_DoseMSG.MaxDoseRate);
    
    gs_Dose.P1 = FloatToSmall(SysRunState.s_DoseMSG.P1);
    gs_Dose.P2 = FloatToSmall(SysRunState.s_DoseMSG.P2);
    
    SendData('V',(uint8_t*)&gs_Dose,sizeof(STU_DOSERATE));
}


/*******************************************************************************
功能：写报警参数命令(WF)响应
输入：unsigned char *cdata
输出：无
*******************************************************************************/
void ACK_CMD_B(u8 *cdata)
{
    SYS_ALARM wal;   
    memcpy((uint8_t*)&wal,cdata,sizeof(SYS_ALARM));

    SysRunState.stParam.s_Alarm.DoseAlarm = FloatToSmall(wal.DoseAlarm);
    SysRunState.stParam.s_Alarm.DosePreAlarm = FloatToSmall(wal.DosePreAlarm);
    SysRunState.stParam.s_Alarm.DoseRateAlarm = FloatToSmall(wal.DoseRateAlarm);
    SysRunState.stParam.s_Alarm.DoseRatePreAlarm = FloatToSmall(wal.DoseRatePreAlarm);

    SendData('B',NULL,0);
    
    //SaveParam();

} 


/*******************************************************************************
功能：读报警参数命令(F)响应
输入：无
输出：无
*******************************************************************************/
void ACK_CMD_F(void)
{
    SYS_ALARM ral;
    ral.DoseAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseAlarm);
    ral.DosePreAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DosePreAlarm);
    ral.DoseRateAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRateAlarm);
    ral.DoseRatePreAlarm = FloatToSmall(SysRunState.stParam.s_Alarm.DoseRatePreAlarm);

    SendData('F',(uint8_t*)&ral,sizeof(SYS_ALARM));
} 

/*******************************************************************************
功能：工作状态
输入：
输出：
*******************************************************************************/
void ACK_CMD_E(void)
{
    uint16_t state = 3;
    uint16_t m_state = WordToSmall(state);
    SendData('E',(uint8_t*)&m_state,2);
}

/*******************************************************************************
功能：清除累计剂量(1)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_ClearDoseSum(void)
{
    SysRunState.s_DoseMSG.Dose = 0;
    //SysRunState.s_DoseMSG.Dose_B = 0;
    SendData(1,NULL,0);
}



/*******************************************************************************
功能：清除最大剂量率(2)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_ClearMaxDoseRate(void)
{
    SysRunState.s_DoseMSG.MaxDoseRate = 0;
    //SysRunState.s_DoseMSG.MaxDoseRate_B = 0;
    SendData(2,NULL,0);
}

/*******************************************************************************
功能：探测器自检(3)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SelfCheck(void)
{
    //自检
      uint16_t result=1;
    uint16_t m_result;
    if(/*(Get_Low_Counter() < LOW_DIBENDI)||*/(Get_Low_Counter() > LOW_GAOBENDI)
       /*||(Get_High_Counter() < HIGH_DIBENDI)*/||(Get_High_Counter() > HIGH_GAOBENDI))
    {
          result = 0;
    }
     m_result = WordToSmall(result); 
    SendData(3,(uint8_t*)&m_result,2);
}

/*******************************************************************************
功能：开关探测器(4)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SensorONOFF(uint16_t state)
{
    SendData(4,NULL,0);
    //开关探测器
    if(state == 1)
    {
        SensorInit();
    }
    else
    {
          //DevSleep();
          SysRunState.NoUartTime = POWER_OFF_TIME+1;
    }
}

/*******************************************************************************
功能：电池电量查询(5)响应
输入：
输出：
*******************************************************************************/
STU_BATTERY s_Bat={0};
void ACK_CMD_Bat(void)
{
    STU_BATTERY bat;
    
    DeviceGetBatAlarm(&s_Bat);
    
    bat.Voltage = WordToSmall(s_Bat.Voltage);
    bat.Status = s_Bat.Status;
    bat.batPercent = s_Bat.batPercent; 
    
    SendData(5,(uint8_t*)&bat,sizeof(STU_BATTERY));
}

/*******************************************************************************
功能：报警确认(6)响应
输入：
输出：
*******************************************************************************/
void ACK_CMD_SureAlarm(void)
{
    //报警确认，关闭相应指示灯
    
    SendData(6,NULL,0);
}


/*******************************************************************************
功能：切换高低量程
输入：
输出：
*******************************************************************************/
void ACK_CMD_GmSw(unsigned char Gm)
{
    if (Gm == 1)
    {
        GDoseSeg = HIG_SEG;
    }
    else
    {
        GDoseSeg = LOW_SEG;
    }
    
    SendData(7,NULL,0);
}

//========================================================================
// 函数名称: WORD CheckSum(BYTE *buf, WORD len)
// 函数功能: 校验和函数
// 入口参数: @*buf：数据；@len：长度
// 函数返回: 校验结果
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================
WORD CheckSum(BYTE *buf, WORD len)
{
    WORD dwSum = 0;
    WORD i;

    for (i = 0; i < len; i++)
    {
        dwSum += buf[i];
    }
    return dwSum;
}
    
void SaveParam()
{
    EA = 0;//禁止中断
    WritePara();
    EA = 1;//允许中断
}

