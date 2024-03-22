#include <math.h>
#include <main.h>
#include "system.h"
#include "Sensor.h"
#include "CalcDoseRate.h"

float CpsToUsv_h(float parama, float paramb, float paramc, float count)
{   
    float ret;
    float c = count;
    float c2 = c*c;
    float c3 = c2*c;
    ret = parama + paramb * c + paramc * c2 + CanshuD * c3;

    return ret;
}