#include "Sensor.h"
#include "CalcDoseRate.h"
#include "CalcCPS.h"
#include "DoseRate.h"
#include "system.h"
#include "CMD.h"

u8 GDoseSeg = LOW_SEG;       //当前处在的段
u8 Dose_switch = 0;    //低量程段0：测量值为γ，1：测量值为β
u32 Low_CPS = 0;
u32 High_CPS = 0;
static float HightDoseRate;
static float LowDoseRate;

static float LowSmothCPS,HighSmothCPS;
static float LowSmothCPS2,HighSmothCPS2;

static float LowNOSmothCPS,HighNOSmothCPS;
static float LowSumCPS,HighSumCPS;

//β数据
static float HSumCPS;

static float SmothCPS_B;
static float NOSmothCPS_B;
static float SumCPS_B;

u32 InSenserCnt = 0;
float OldDr = 0.0;
float NewDr;
float RtCps,NewCps;
float CanshuA,CanshuB,CanshuC,CanshuD;
void SensorInit(void)
{
    //CanshuA = SysRunState.stParam.s_SysParam.DiYaCanshuA;
    //CanshuB = SysRunState.stParam.s_SysParam.DiYaCanshuB;
    //CanshuC = SysRunState.stParam.s_SysParam.DiYaCanshuC; 
    //CanshuD = 0.0;

    UseSLParam(0);
    memset((void*)&SysRunState.s_DoseMSG,0,sizeof(STU_DOSERATE));
}

void SensorMeasureBegin(void)
{ 
    Low_CPS = 0;
    High_CPS = 0;
    //GetCounter();
    //GetBataCounter();
}

void UseSLParam(float dr)
{
    if (dr < 8)
    {
        CanshuA = 1.06581410364015E-14; // SysRunState.stParam.s_SysParam.DiYaCanshuA;
        CanshuB = 0.291478787484166;     // SysRunState.stParam.s_SysParam.DiYaCanshuB;
        CanshuC = 0.0108551044292504; //SysRunState.stParam.s_SysParam.DiYaCanshuC; 
        CanshuD = 0.0;
    }
    else if (dr < 500)
    {
        CanshuA = -0.358286803618626;
        CanshuB = 0.494100392430478;
        CanshuC = 9.76367631615528E-05;
        CanshuD = 0.0;

    }
    else  if (dr < 8000)
    {
        CanshuA = 125.037524058913;
        CanshuB = 0.305815634950024;
        CanshuC =  0.000147696929666487;
        CanshuD = 0.0;
    }
    else
    {
        CanshuA = -1129.62834067371;
        CanshuB = 1.44007488389396;
        CanshuC =  0.0;
        CanshuD = 0.0;
    }
    
    
}


void UseSHParam(float dr)
{
    if (dr < 80000)
    {
        CanshuA = -56.4956084021251;
        CanshuB = 33.1391426920277;
        CanshuC =  -0.00121629383397334;
        CanshuD = 0.0;
    }
    else
    {
        CanshuA = -9890.61062326864;
        CanshuB = 32.8126304817507;
        CanshuC = 0.000275557442490988;
        CanshuD = 0.0;
    }
}


float DrFix(float dr)
{
    float fix = 1.0;
    if (dr < 1.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[0]);
    }
    else if (dr < 10.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[1]);
    }
    else if (dr < 100.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[2]);
    }
    else if (dr < 1000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[3]);
    }
    else if (dr < 10000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[4]);
    }
    else if (dr < 100000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[5]);
    }
    else if (dr < 1000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[6]);
    }
    else if (dr < 10000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[7]);
    }
    else if (dr < 100000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[8]);
    }
    else if (dr < 1.0E+9) //1000000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[9]);
    }
    else if (dr < 1.0E+10) // 10000000000.0)
    {
        fix = FloatToSmall(SysRunState.stParam.Fix[10]);
    }

    return (dr*fix);
}



void CaptureSensorPluseCounter(void)
{
    /**************测试用的****************************************/
    //Low_CPS = 10;
    //High_CPS = 10;
    /*****************************************************/
    
    //FilterLow(Low_CPS);
    //FilterHigh(High_CPS);
    //LowSumCPS += Low_CPS;

    LowSumCPS = GetCounter();
    //HighSumCPS += High_CPS;
    HighSumCPS = GetHightCounter();

    if((LowSumCPS == 0)&&(SysRunState.LowChanneloff == 0))
    {
          SysRunState.LChannelNoCountTime++;
    }
    else
    {
          SysRunState.LChannelNoCountTime = 0;
    }
    if(HighSumCPS == 0)
    {
          SysRunState.HChannelNoCountTime++;
    }
    else
    {
          SysRunState.HChannelNoCountTime = 0;
    }
    switch(GDoseSeg)
    {
        case LOW_SEG:
        {
            SysRunState.LowChanneloff = 0;
            LowSmothCPS = CalcLow(
                        CanshuA, 
                        CanshuB, 
                        CanshuC,
                        LowSumCPS, 
                        LowDoseRate,
                        &SysRunState.s_DoseMSG.C1);
            if (LowSmothCPS != -1)
            {
                SysRunState.s_DoseMSG.C1 *= SysRunState.stParam.s_SysParam.DiYaCanshuC;
                LowDoseRate = SysRunState.s_DoseMSG.C1;
            }
            UseSLParam(SysRunState.s_DoseMSG.C1);
            SysRunState.s_DoseMSG.DoseRate = LowDoseRate;

            if (SysRunState.s_DoseMSG.DoseRate >= USE_LOW_USV)// &&
                  //(SysRunState.s_DoseMSG.C2 >= 30)  ) // 确保是伽马才切换
            {
                GDoseSeg = HIG_SEG;
                ClearCounter();
            }
            break;
        }

        case HIG_SEG:
        {
            SysRunState.LowChanneloff = 1;
            HighSmothCPS = CalcHigh(
                      CanshuA, 
                      CanshuB, 
                      CanshuC,
                      HighSumCPS, 
                      HightDoseRate,
                      &SysRunState.s_DoseMSG.C2);
            if (HighSmothCPS != -1)
            {
                SysRunState.s_DoseMSG.C2 *= SysRunState.stParam.s_SysParam.GaoYaCanshuC;
                HightDoseRate = SysRunState.s_DoseMSG.C2;
            }
            UseSHParam(HightDoseRate);
            SysRunState.s_DoseMSG.DoseRate = HightDoseRate;

            if(SysRunState.s_DoseMSG.DoseRate < USE_HIGH_USV)
            {
                GDoseSeg = LOW_SEG;
                ClearCounter();
            }
            break;
        }
        default: GDoseSeg = LOW_SEG;break;
        
    }
                 
    LowNOSmothCPS = LowSumCPS;
    HighNOSmothCPS = HighSumCPS;    
        
    HighSumCPS = 0;
    LowSumCPS = 0;
    
    /*if(SysRunState.s_DoseMSG.C1 > 1)
    {
        //剂量率大于1，停止
        LowSumCPS = 0;
    }*/
    
    SysRunState.s_DoseMSG.P1 = LowNOSmothCPS;
    SysRunState.s_DoseMSG.P2 = HighNOSmothCPS;

    /*if(SysRunState.testtime>0)
    {
          SysRunState.s_DoseMSG.DoseRate = 999.9;
    }*/

    SysRunState.s_DoseMSG.DoseRate = DrFix(SysRunState.s_DoseMSG.DoseRate);
    SysRunState.s_DoseMSG.Dose += SysRunState.s_DoseMSG.DoseRate/3600.0f;
    
    if(SysRunState.s_DoseMSG.DoseRate>SysRunState.s_DoseMSG.MaxDoseRate)
    {
        SysRunState.s_DoseMSG.MaxDoseRate = SysRunState.s_DoseMSG.DoseRate;
    }
    
    CalcAlarmState(&SysRunState);           

}

float Get_Low_Counter(void)
{
    return LowNOSmothCPS;
}

float Get_High_Counter(void)
{
    return HighNOSmothCPS;
}

float Get_Low_Smooth_Counter(void)
{
    return LowSmothCPS;
}

float Get_High_Smooth_Counter(void)
{
    return HighSmothCPS;
}

u16 CalcAlarmState(LP_SYSTEM_STTAE *me)
{
#if 0
      /* 剂量当量报警检查 */    
    if ((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DoseAlarm)&&(me->stParam.s_Alarm.DoseAlarm > 0)) 
    { 
        me->s_DoseMSG.DoSt = 2;
    } 
    /* 剂量当量预警检查 */    
    else if((me->s_DoseMSG.Dose >= me->stParam.s_Alarm.DosePreAlarm)&&(me->stParam.s_Alarm.DosePreAlarm > 0)) 
    { 
        me->s_DoseMSG.DoSt = 1;
    } 
#endif
    
    //U16 alarmState = me->Alarmstate&BATTARY_LOW_BIT;
      if(me->s_DoseMSG.DoseRate >= 9999999)//10Sv以上则是过载报警
    {
          me->s_DoseMSG.DoseRate = 9999999;
        me->s_DoseMSG.DRSt = 5;
        return true;
    }
    
    /* 剂量当量率报警检查 */    
    if ((me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRateAlarm)&&(me->stParam.s_Alarm.DoseRateAlarm > 0)) 
    { 
        if((++me->DoseRateAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_ALARM) {//连续两次报警则认为报警
            me->s_DoseMSG.DRSt = 2;
            return true;
        }
    } else {
        me->DoseRateAlarmCnt= 0x0;
        me->s_DoseMSG.DRSt = 0;
    }
    
    /* 剂量当量率预警检查 */    
    if ((me->s_DoseMSG.DoseRate >= me->stParam.s_Alarm.DoseRatePreAlarm)&&(me->s_DoseMSG.DoseRate < me->stParam.s_Alarm.DoseRateAlarm))
    { 
        if((++me->DoseRatePreAlarmCnt) >= MIB_CST_DOSERATE_THRESHOLD_WARNING) {//连续两次报警则认为报警
            me->s_DoseMSG.DRSt = 1;
            return true;
        }
    } else {
        me->DoseRatePreAlarmCnt= 0x0;
        me->s_DoseMSG.DRSt = 0;
    }
    
    //if((SysRunState.LChannelNoCountTime>60)&&(SysRunState.HChannelNoCountTime>1200))//低通道1分钟无数据，高通道10分钟无数据则报双探测器异常
    //{
    //      me->s_DoseMSG.DRSt = 8;
    //}
    //else 
    if(SysRunState.LChannelNoCountTime>60)//低通道1分钟无数据,探测器异常
    {
          me->s_DoseMSG.DRSt = 6;
    }
    //else if(SysRunState.HChannelNoCountTime>1200)//高通道20分钟无数据,探测器异常
    //{
          //me->s_DoseMSG.DRSt = 7;
    //}
    else
    {
          me->s_DoseMSG.DRSt = 0;
    }
    return true;
}


