#include <main.h>
#include <math.h>
#include "DoseRate.h"
#include "CalcCPS.h"
#include "sensor.h"

//extern QInnerParam InnerParam;


#ifdef  USE_LP_DOSECALC

float OldDr = 0.0;
float NewDr;
float RtCps,NewCps;

void CpsFilter(DWORD Cps)
{
    RtCps = (float)Cps;
}

float CounterPH()
{
    NewCps = CalcLow(RtCps, OldDr, &NewDr);
    if (NewDr > 9999)
    {
        return NewCps;
    }
    
    OldDr = NewDr;
    return NewCps; // NewDr;
}

void ClearCounter()
{
    InitCalcLow();
}


#else

// 取平均值
#define CT 60   
DWORD CpsArr[2][CT];

static BOOL Full = FALSE;
static BYTE ArrCnt = 0;
static float CpsHis = 0;

void InitArr()
{
    BYTE i,j;
    // 初始状态填15 个  1
    memset(CpsArr,0,sizeof(CpsArr));
    for(j = 0;j < 2;j++)
    {
        for (i=0;i<CT;i+=4)
        {
            CpsArr[j][i] = 1;
        }
    }
    Full = TRUE;
    ArrCnt = CT;
    CpsHis = 15.0/CT;
}

// 将计数压入堆栈
void PushCps(DWORD cps)
{
    BYTE i;
    if(GDoseSeg == LOW_SEG)
    {
        if (ArrCnt >= CT)
        {
            Full = TRUE;
            for (i=1;i<CT;i++)
            {
                CpsArr[0][i-1] = CpsArr[0][i];
            }
            CpsArr[0][CT-1] = cps;
        }
        else
        {
            CpsArr[0][ArrCnt++] = cps;
        }
    }

    if(GDoseSeg == HIG_SEG)
    {
        if (ArrCnt >= CT)
        {
            Full = TRUE;
            for (i=1;i<CT;i++)
            {
                CpsArr[1][i-1] = CpsArr[1][i];
            }
            CpsArr[1][CT-1] = cps;
        }
        else
        {
            CpsArr[1][ArrCnt++] = cps;
        }
    }
}

void ClearCounter()
{
    memset(CpsArr,0,sizeof(CpsArr));
    Full = FALSE;
    ArrCnt = 0;
    CpsHis = 0;
}


// 计数平滑
float CounterPH()
{
    float Val = 0.0;
    BYTE i;
    Val = 0.0;
    if(GDoseSeg == LOW_SEG)
    {
        
        if (Full)
        {
            for (i=0;i<CT;i++)
            {
                Val += CpsArr[0][i];
            }
            Val = Val/CT;

        }
        else
        {
            if (ArrCnt == 0)
            {
                CpsHis = 0.2;
                return -1;
            }
            else
            {
                for (i=0;i<ArrCnt;i++)
                {
                    Val += CpsArr[0][i];
                }
                Val = Val/ArrCnt;
            }
        }

        if (Val < 0.2)
        {
            Val = 0.2;
        }
    }

    
    if(GDoseSeg == HIG_SEG)
    {
        if (Full)
        {
            for (i=0;i<CT;i++)
            {
                Val += CpsArr[1][i];
            }
            Val = Val/CT;

        }
        else
        {
            if (ArrCnt == 0)
            {
                CpsHis = 0.2;
                return -1;
            }
            else
            {
                for (i=0;i<ArrCnt;i++)
                {
                    Val += CpsArr[1][i];
                }
                Val = Val/ArrCnt;
            }
        }

        if (Val < 0.2)
        {
            Val = 0.2;
        }
    }
    
    CpsHis = Val;
    return Val;
}


float GetHis(BYTE time)
{
    BYTE i,j;
    float ret = 0.0;
    if (Full)
    {
        for(j = 0;j<2;j++)
        {
            for (i=CT-time;i<CT;i++)
            {
                ret += CpsArr[j][i];
            }
        }
        return (ret/time);
    }
    else
    {
        if (time > ArrCnt)
        {
            for(j = 0;j<2;j++)
            {
                for (i=0;i<ArrCnt;i++)
                {
                    ret += CpsArr[j][i];
                }
            }
            return (ret/ArrCnt);
        }
        else
        {
            for(j = 0;j<2;j++)
            {
                for (i=ArrCnt-time;i<ArrCnt;i++)
                {
                    ret += CpsArr[j][i];
                }
            }
            return (ret/time);
        }
    }
    //return 0.0;
}


// time :  保留最后计数的时间
void ResvCps(BYTE time)
{
    BYTE i;
    if(GDoseSeg == LOW_SEG)
    {
        if (Full)
        {
            for (i=0;i<CT;i++)
            {
                if (i<time)
                {
                    CpsArr[0][i] = CpsArr[0][CT-time+i];
                }
                else
                {
                    CpsArr[0][i] = 0;
                }
            }
            ArrCnt = time;
            Full = FALSE;
        }
        else
        {
            if (time < ArrCnt)
            {
                for (i=0;i<ArrCnt;i++)
                {
                    if (i<time)
                    {
                        CpsArr[0][i] = CpsArr[0][ArrCnt-time+i];
                    }
                    else
                    {
                        CpsArr[0][i] = 0;
                    }
                }

                ArrCnt = time;
            }
            else
            {
                // 全部保留，就不做处理了
            }
        }
    }

    if(GDoseSeg == HIG_SEG)
    {
        if (Full)
        {
            for (i=0;i<CT;i++)
            {
                if (i<time)
                {
                    CpsArr[1][i] = CpsArr[1][CT-time+i];
                }
                else
                {
                    CpsArr[1][i] = 0;
                }
            }
            ArrCnt = time;
            Full = FALSE;
        }
        else
        {
            if (time < ArrCnt)
            {
                for (i=0;i<ArrCnt;i++)
                {
                    if (i<time)
                    {
                        CpsArr[1][i] = CpsArr[1][ArrCnt-time+i];
                    }
                    else
                    {
                        CpsArr[1][i] = 0;
                    }
                }

                ArrCnt = time;
            }
            else
            {
                // 全部保留，就不做处理了
            }
        }
    }
}

// 计数压入队列
void AddCounter(DWORD Cps)
{
    #if 0
    static float His120S = 0.0;
    //static float His60S  = 0.0;
    static float His30S  = 0.0;
    static float His10S  = 0.0;
    static float His5S   = 0.0;
    static float His3S   = 0.0;

    float M1,M2,M3;
    #endif
    
    PushCps(Cps);

    #if 0
    His120S = GetHis(120);
    //His60S  = GetHis(60);
    His30S  = GetHis(30);
    His10S  = GetHis(10);
    His5S   = GetHis(5);
    His3S   = GetHis(3);

    if (InnerParam.DevType == 4)
    {
        M1 = 30;   // 1mSv/h
        M2 = 2.5;  // 90uSv/h
        M3 = 1.5;  // 30uSv/h
    }
    else
    {
        M1 = 1000; // 0.8mSv/h
        M2 = 80;   // 50uSv/h
        M3 = 1.6;  // 1uSv/h
    }

    if (His3S > M1)   
    {
        if ((His3S > His120S * 1.03) || (His3S < His120S / 1.03))
        {
            ResvCps(3);
        }
    }
    else if (His5S > M2)  
    {
        if ((His5S > His120S * 1.1) || (His5S < His120S / 1.1))
        {
            ResvCps(5);
        }
    }
    else if (His10S > M3)  
    {
        if ((His10S > His120S * 1.2) || (His10S < His120S / 1.2))
        {
            ResvCps(10);
        }
    }
    else if (His30S > 0.0)  
    {
        if ((His30S > His120S * 1.3) || (His30S < His120S / 1.3))
        {
            ResvCps(30);
        }
    }

    #if 0
    else if (His60S > 0.0)  // 0.0u
    {
        if ((His60S > His120S * 1.1) || (His60S < His120S / 1.1))
        {
            ResvCps(60);
        }
    }
    #endif

    #endif
    
}


void CpsFilter(DWORD Cps)
{
    static DWORD BigCpsErr[3] = {0};
    static BYTE BigErrCnt = 0;

    static DWORD SmCpsErr[3] = {0};
    static BYTE SmErrCnt = 0;
    
    //if (CpsHis > 1.0)
    {
        //if (Cps > (CpsHis * 3))  // 大计数
        if(Cps>(CpsHis+3*sqrt(CpsHis)))
        {
            SmErrCnt = 0;
            BigCpsErr[BigErrCnt++] = Cps;
            if (BigErrCnt == 3)
            {
                // 连续两个大计数则不算抖动
                ClearCounter();
                AddCounter(BigCpsErr[0]);
                AddCounter(BigCpsErr[1]);
                AddCounter(BigCpsErr[2]);
                BigErrCnt = 0;
            }
        }
        //else if (Cps < (CpsHis / 3))  // 小计数
        else if(Cps<(CpsHis-3*sqrt(CpsHis)))
        {
            BigErrCnt = 0;
            SmCpsErr[SmErrCnt++] = Cps;
            if (SmErrCnt == 3)
            {
                // 连续两个小计数则不算抖动
                ClearCounter();
                AddCounter(SmCpsErr[0]);
                AddCounter(SmCpsErr[1]);
                AddCounter(SmCpsErr[2]);
                SmErrCnt = 0;
            }
        }
        else
        {
            AddCounter(Cps);
            BigErrCnt = 0;
            SmErrCnt = 0;
        }
    }
    #if 0
    else
    {
        AddCounter(Cps);
        BigErrCnt = 0;
        SmErrCnt = 0;
    }
    #endif
}

#endif

