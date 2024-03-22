#include "Queue.h"
#include "main.h"
#include "CalcDoseRate.h"
#include "DoseRate.h"
//#define ZGC_DOSE

#define CPS_AVG_BUG_SIZE (2)
#define CPS_COEFFICIENT (1)

unsigned long lCPS = 0;
unsigned long lCP100MS = 0;

// 考虑到分开处理和后续可能同时测量，分开使用过滤buffer
//static squeue stFilterQ;
//static squeue stCPSQ;

//static squeue stFilterAlphaQ;
//static squeue stAlphaQ;

//static squeue stFilterBetaQ;
//static squeue stBetaQ;

// 平滑buffer
//static squeue stAlphaCPSAvgQ,stBetaCPSAvgQ;

//static S_BENDI gs_AlphaBendi,gs_BetaBendi;

//static void Filter(squlink FilterQ, squlink CalcQ, unsigned long nNewCnt);
#ifdef ZGC_DOSE
static DataType SmoothCps(float parama, float paramb, float paramc, float RealCPS, squlink avgQ, float OlduSvh, S_BENDI *bendi, float *NewuSvh);


/******************************************************
Cps 过滤

void InitCalcCps(void)
{
    InitQueue(&stFilterQ, (FILTER_BUF_SIZE));
    InitQueue(&stCPSQ, (CPS_BUF_SIZE));
    InitQueue(&stCPSAvgQ, (CPS_AVG_BUG_SIZE));
}

// 每一秒被调用一次
unsigned long CalcCps(void)
{
    DataType CPS = 0;
    //CPS = SmoothCps(&stCPSQ);
    return CPS;
}*/
/*
void EmtyCalcCpsQ(void)
{
    InitQueue(&stCPSQ, (CPS_BUF_SIZE));
    InitQueue(&stCPSAvgQ, (CPS_AVG_BUG_SIZE));
}

// 每100ms调用一次
void FilterCps(unsigned long nNewCnt)
{
    Filter(&stFilterQ, &stCPSQ, nNewCnt);
}

// 读取N个点（N*100ms）计算趋势CPS来判断是否
// 返回CPS
unsigned long  CalcTrendCps(void)
{
    // 100ms * CALC_TREND_CPS_CNT
    unsigned long CPS = SumRear(&stCPSQ, CALC_TREND_CPS_CNT);
    // 1000ms
    float fCoEfficient = 1000.0f/(CALC_TREND_CPS_CNT*100.0F);
    CPS *= fCoEfficient;
    CPS *= CPS_COEFFICIENT;
    
    return CPS;
}
*/

/******************************************************
Low过滤
*/
void InitCalcLow(void)
{
    //InitQueue(&stFilterAlphaQ, (FILTER_BUF_SIZE));
    //InitQueue(&stAlphaQ, (CPS_BUF_SIZE));
    //InitQueue(&stAlphaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}


void EmtyCalcLowQ(void)
{
    //InitQueue(&stAlphaQ, (CPS_BUF_SIZE));
    InitQueue(&stAlphaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}

void EmtyBendi(void)
{
    memset((void*)&gs_AlphaBendi,0,sizeof(S_BENDI));
    memset((void*)&gs_BetaBendi,0,sizeof(S_BENDI));
	gs_AlphaBendi.OlduSvh2 = 0.5;
	gs_AlphaBendi.OlduSvh3 = 0.5;
	gs_AlphaBendi.OlduSvh4 = 0.5;
	gs_AlphaBendi.OlduSvh5 = 0.5;
	gs_AlphaBendi.OlduSvh6 = 0.5;
	gs_BetaBendi.OlduSvh2 = 0.5;
	gs_BetaBendi.OlduSvh3 = 0.5;
	gs_BetaBendi.OlduSvh4 = 0.5;
	gs_BetaBendi.OlduSvh5 = 0.5;
	gs_BetaBendi.OlduSvh6 = 0.5;
}
#endif

// 每一秒被调用一次,输入参数，返回CPS，带出剂量率
DataType CalcLow(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh)
{
    DataType CPS = 0;
    #ifdef ZGC_DOSE
    CPS = SmoothCps(parama, paramb, paramc, RealCPS, &stAlphaCPSAvgQ, OlduSvh, &gs_AlphaBendi, NewuSvh);
    #else
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH();
    if (CPS != -1)
    {
        *NewuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
    }
    #endif
    /*if(*NewuSvh > 1)
	{
	  	return *NewuSvh;
	}*/
    return CPS;
}

#ifdef ZGC_DOSE

// 每100ms调用一次
void FilterLow(unsigned long nNewCnt)
{
    //Filter(&stFilterAlphaQ, &stAlphaQ, nNewCnt);
}

/******************************************************
High过滤
*/
void InitCalcHigh(void)
{
    //InitQueue(&stFilterBetaQ, (FILTER_BUF_SIZE));
    //InitQueue(&stBetaQ, (CPS_BUF_SIZE));
    InitQueue(&stBetaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}

void EmtyCalcHighQ(void)
{
    //InitQueue(&stBetaQ, (CPS_BUF_SIZE));
    InitQueue(&stBetaCPSAvgQ, (CPS_AVG_BUG_SIZE-1));
}
#endif

// 每一秒被调用一次
DataType CalcHigh(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh)
{
    DataType CPS = 0;
    #ifdef ZGC_DOSE
    CPS = SmoothCps(parama, paramb, paramc, RealCPS, &stBetaCPSAvgQ, OlduSvh, &gs_BetaBendi, NewuSvh);
    #else
    CpsFilter((DWORD)RealCPS);    
    CPS = CounterPH();
    if (CPS != -1)
    {
        *NewuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
    }
    #endif
    return CPS;
}


// 每100ms调用一次
void FilterHigh(unsigned long nNewCnt)
{
    //Filter(&stFilterBetaQ, &stBetaQ, nNewCnt);
}

//static float AllTimeCPS,AllTimeSec;//长时间累加的计数和时间
/******************************************************
平滑算法
//squlink Q 对象
*/

#ifdef ZGC_DOSE

#define TURNINGUSV      		(10)//转折剂量率，usv
#define LOW_UPFAST      		(5)//低剂量变化倍数
#define HIGH_UPFAST     		(3)//高剂量变化倍数
#define LOW_UPDOWN_TIMES  		(2)//低剂量响应时间
#define HIGHT_UPDOWN_TIMES  	(1)//连续突变取数


DataType SmoothCps(float parama, float paramb, float paramc, float RealCPS, squlink avgQ, float OlduSvh, S_BENDI *bendi, float *NewuSvh)
{
    static DataType CPS = 0,ave=0;
    unsigned long Cnt;// = Lenqueue(Q);
    float uSvh,tempuSvh,realTimeCPSuSvh;
    float multiple;
	unsigned char updowntimes;


    if(OlduSvh < 0.5)
    {
        OlduSvh = 0.5;
    }
    bendi->OlduSvh6 = bendi->OlduSvh5;
    bendi->OlduSvh5 = bendi->OlduSvh4;
    bendi->OlduSvh4 = bendi->OlduSvh3;
    bendi->OlduSvh3 = bendi->OlduSvh2;
    bendi->OlduSvh2 = OlduSvh;
    //CPS = SumRear(Q, Cnt);
    CPS = RealCPS;//读取最新一个数据

    EnterQueue(avgQ, CPS);//先存入平均值数据队列中

    Cnt = Lenqueue(avgQ);

    ave = CPS;//SumRear(avgQ, 2)/2;

    //判断是否陡升陡降
    uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
	if(uSvh > 100000)//10m以上用实时值
	{	
	  	*NewuSvh = uSvh;
		return ave; 
	}
    tempuSvh = uSvh;
    if(tempuSvh < 0.5)
    {
        tempuSvh = 0.5;
    }
    if((tempuSvh < TURNINGUSV)&&(bendi->OlduSvh3 < TURNINGUSV))//10u以下5倍变化
    {
        multiple = LOW_UPFAST;
		updowntimes = LOW_UPDOWN_TIMES;
    }
    else
    {
        multiple = HIGH_UPFAST;
		updowntimes = HIGHT_UPDOWN_TIMES;
    }
    if((tempuSvh > multiple*bendi->OlduSvh3)||(tempuSvh * multiple < bendi->OlduSvh3))
    {
        bendi->updownTimes++;
        if(bendi->updownTimes >= 2)
        {
            bendi->updownTimes = 0;
            if(Cnt >=updowntimes)
            {
                SetQLength(avgQ, updowntimes);
                Cnt = updowntimes;
            }
            ave = SumRear(avgQ, Cnt)/Cnt;//前Cnt秒平均值
            uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
            *NewuSvh = uSvh;
			
            bendi->down30 = 0;
            bendi->startFlashFlag = 0;
            bendi->FlashTimes = 0;
            bendi->OlduSvh6 = uSvh;
            bendi->OlduSvh5 = uSvh;
            bendi->OlduSvh4 = uSvh;
            bendi->OlduSvh3 = uSvh;
            bendi->OlduSvh2 = uSvh;
			bendi->time = Cnt;
			bendi->cps = ave*Cnt;
            return ave;     
        }
    }
    else
    {
        bendi->updownTimes = 0;
    }

    if(Cnt >= 3)
    {
        ave = SumRear(avgQ, 3)/3;//前3秒平均值
        uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
        if((uSvh > 1000.0)&&(abs(100.0*(uSvh-bendi->OlduSvh3)/(bendi->OlduSvh3+1.0)) > 6))//3秒计数大于1m，并且波动大于上次平均的10%，用前3秒的数据
        {
            SetQLength(avgQ, 3);
            *NewuSvh = uSvh;

			bendi->time = 3;
			bendi->cps = ave*3;

            bendi->OlduSvh6 = uSvh;
            bendi->OlduSvh5 = uSvh;
            bendi->OlduSvh4 = uSvh;
			bendi->OlduSvh3 = uSvh;
            bendi->OlduSvh2 = uSvh;
            return ave;
        }
    }

   
    if(Cnt >= 5)
    {
        realTimeCPSuSvh = CpsToUsv_h(parama, paramb, paramc, CPS);
        if(realTimeCPSuSvh < 0.5)
        {
            realTimeCPSuSvh = 0.5;
        }
        if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 50)/*&&(realTimeCPSuSvh >= 1)*/)//上升
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }

        }
        else if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 28)&&(realTimeCPSuSvh >= 3))//上升
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(realTimeCPSuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1)) > 20)&&(realTimeCPSuSvh  >= 8))//上升
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 50)/*&&(bendi->OlduSvh5 >= 1)*/)//下降
        {
            bendi->down30++;
            bendi->startFlashFlag = 1;
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 28)&&(bendi->OlduSvh5 >= 3))//下降
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else if(((100.0*(bendi->OlduSvh5 - realTimeCPSuSvh)/(realTimeCPSuSvh+0.1)) > 20)&&(bendi->OlduSvh5 >= 8))//下降
        {
            bendi->startFlashFlag = 1;
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
        }
        else
        {
            if(bendi->down30 > 0)
            {
                bendi->down30--;
            }
            
        }
        if(bendi->down30 >= 9)
        {
            bendi->down30 = 0;
            if(bendi->OlduSvh5 < 1.5)
			{
			  	bendi->startFlashFlag = 0;
				bendi->FlashTimes = 0;
				
				SetQLength(avgQ, 5);
				ave = SumRear(avgQ, 5)/5;
				uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
				*NewuSvh = uSvh;
				bendi->OlduSvh6 = uSvh;
				bendi->OlduSvh5 = uSvh;
				bendi->OlduSvh4 = uSvh;
				bendi->OlduSvh3 = uSvh;
				bendi->OlduSvh2 = uSvh;
				bendi->time = 5;
				bendi->cps = ave*5;
				return ave;
			}
        }
        
        if(bendi->startFlashFlag == 1)
        {
            bendi->FlashTimes++;
        }
        else
        {
            bendi->FlashTimes = 0;
        }
        if(bendi->FlashTimes >= 5)
        {
            ave = SumRear(avgQ, 5)/5;//前5秒平均值
            uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
			tempuSvh = uSvh;
			if(tempuSvh < 0.5)
			{
			  	tempuSvh = 0.5;
			}
			if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 10))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 10)))//波动大于上次平均的30%，用前5秒的数据
            {
                SetQLength(avgQ, 5);
                *NewuSvh = uSvh;
                bendi->down30 = 0;
                bendi->OlduSvh6 = uSvh;
                bendi->OlduSvh5 = uSvh;
                bendi->OlduSvh4 = uSvh;
                bendi->OlduSvh3 = uSvh;
                bendi->OlduSvh2 = uSvh;
				
				bendi->startFlashFlag = 0;
            	bendi->FlashTimes = 0;
				bendi->time = 5;
				bendi->cps = ave*5;
                return ave;
            }
			Cnt = Lenqueue(avgQ);
			if(Cnt >= 10)
			{
			  	bendi->startFlashFlag = 0;
            	bendi->FlashTimes = 0;
				ave = SumRear(avgQ, 10)/10;//前10秒平均值
            	uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
				tempuSvh = uSvh;
				if(tempuSvh < 0.5)
				{
					tempuSvh = 0.5;
				}
				if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 80)))//
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 80))))//波动大于上次平均的30%，用前5秒的数据
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 50))&&(bendi->OlduSvh5 > 1.5))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 50))&&(bendi->OlduSvh5 > 1.5)))//波动大于上次平均的30%，用前5秒的数据
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 3))//10u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 35))&&(bendi->OlduSvh5 > 3)))//波动大于上次平均的30%，用前5秒的数据
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
				else if((((100*(tempuSvh-bendi->OlduSvh5)/(bendi->OlduSvh5+0.1) > 15))&&(bendi->OlduSvh5 > 100))//100u
                ||(((100*(bendi->OlduSvh5 - tempuSvh)/(bendi->OlduSvh5+0.1) > 15))&&(bendi->OlduSvh5 > 100)))//波动大于上次平均的30%，用前5秒的数据
				{
					SetQLength(avgQ, 5);
					*NewuSvh = uSvh;
					bendi->down30 = 0;
					bendi->OlduSvh6 = uSvh;
					bendi->OlduSvh5 = uSvh;
					bendi->OlduSvh4 = uSvh;
					bendi->OlduSvh3 = uSvh;
					bendi->OlduSvh2 = uSvh;
					bendi->time = 5;
					bendi->cps = ave*5;
					return ave;
				}
			}
        }
    }



    //前面都不满足则用实际所有数据的平均值
	Cnt = Lenqueue(avgQ);
	//连续3秒的计数都大于0.5，
	if((bendi->OlduSvh2 > 30)&&(bendi->OlduSvh3 > 30)&&(bendi->OlduSvh4 > 30))
	{
	  	if(Cnt > 60)
		{
		  	SetQLength(avgQ, 60);
			Cnt = 60;
		}
	  	ave = SumRear(avgQ, Cnt)/Cnt;//前Cnt秒平均值
		bendi->time += 1;
		bendi->cps += CPS;
	}
	else if((bendi->OlduSvh2 > 20)&&(bendi->OlduSvh3 > 20)&&(bendi->OlduSvh4 > 20))
	{
	  	if(Cnt > 90)
		{
		  	SetQLength(avgQ, 90);
			Cnt = 90;
		}
		ave = SumRear(avgQ, Cnt)/Cnt;//前Cnt秒平均值
		bendi->time += 1;
		bendi->cps += CPS;
	}
	else
	{
	  	ave = SumRear(avgQ, Cnt)/Cnt;//前Cnt秒平均值
		bendi->time += 1;
		bendi->cps += CPS;
	}
	/*
    else
	{
		//长时间累加值
		bendi->time += 1;
		bendi->cps += CPS;
		ave = bendi->cps/bendi->time;
	}*/
    uSvh = CpsToUsv_h(parama, paramb, paramc, ave);
    *NewuSvh = uSvh;
	Cnt = Lenqueue(avgQ);
	if(Cnt > 15)
	{
	  	tempuSvh = SumRear(avgQ, 15)/15;
		uSvh = CpsToUsv_h(parama, paramb, paramc, tempuSvh);
	  	if((uSvh + 0.5 < *NewuSvh)&&(*NewuSvh < 1.5))
		{
		  	*NewuSvh = uSvh;
			ave = tempuSvh;
			bendi->cps = tempuSvh*15;
			bendi->time = 15;
			SetQLength(avgQ, 15);
		}
	}
    return ave;
}

#endif

