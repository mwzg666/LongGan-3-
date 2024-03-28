#include <math.h>
#include <main.h>
#include "system.h"
#include "Sensor.h"
#include "CalcDoseRate.h"

#ifdef P2P_NH
// 点对点数据拟合
float CpsToUsv_h(NH_PARAM *Nh, float x)
{
    BYTE i;
    float y;
    float a,b,x1,x2,y1,y2;
    
    for (i=1;i<NH_COUNT;i++)
    {
        if (x < Nh[i].cps)
        {
            x1 = Nh[i-1].cps;
            x2 = Nh[i].cps;
            
            y1 = Nh[i-1].dr;
            y2 = Nh[i].dr;
            break;
        }
    }

    if (i == NH_COUNT) 
    {
        // 计数大于最后一个拟合点，就用最后一个
        x1 = Nh[NH_COUNT-2].cps;
        x2 = Nh[NH_COUNT-1].cps;
        
        y1 = Nh[NH_COUNT-2].dr;
        y2 = Nh[NH_COUNT-1].dr;
    }

    a = (y2 - y1) / (x2 - x1);
    b = y2 - x2 * a;

    y = a*x+b;
    if (y < 0)
    {
        y = 0;
    }

    return y;
}
#else
float CpsToUsv_h(float parama, float paramb, float paramc, float count)
{   
    float ret;
    float c = count;
    float c2 = c*c;
    float c3 = c2*c;
    ret = parama + paramb * c + paramc * c2 + CanshuD * c3;

    return ret;
}
#endif
