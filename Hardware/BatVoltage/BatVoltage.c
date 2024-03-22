#include "main.h"

#define AD_NUM 10
#define WWQ_time_out 0xfff

void Adc_Init()
{
    P_SW2 |= 0x80;
    ADCTIM = 0x3f;      //设置 ADC 内部时序，ADC采样时间建议设最大值
    ADCCFG = 0x2f;      //设置 ADC 时钟为系统时钟/2/16/16
    ADC_CONTR = 0x80;   //使能 ADC 模块
    //EADC = 1;
    _nop_();
    delay_ms(1);

    
}

#if 1
uint16_t arrange(uint16_t *a,uint8_t n);
#endif

//unsigned int ADC16Result = 0; 

uint16_t DeviceGetBatVal(void)
{
    #if 1 
    u8 channel = 1;
    uint16_t i = 0,j = 0, myad[AD_NUM] = {0};
	float    temp = 0.0;
    ADC_RES = 0;
    ADC_RESL = 0;
    
    i = 0;
    while(i < AD_NUM)
    {
        ADC_CONTR |= (1<<0);  //通道1
        ADC_START = 1;                   // Start conversions            
        _nop_();
        _nop_();
        _nop_();
        _nop_();
        _nop_();
        while (!(ADC_FLAG))
        {
            j++;  //防止AD内部电路出错
            if( j == WWQ_time_out)
            {
                return 0;//2v,ad这时可能就不工作了
            } 
        }
        ADC_START = 0; 
        ADC_FLAG = 0;;   // Clear Int
        j = 0; //测量完一次
        myad[i++] = ((uint16_t)(ADC_RES << 8)) | ADC_RESL;
        delay_us(100);
    }
    ADC_START = 0; 
    delay_us(1000);

    //temp = arrange(myad,AD_NUM)*4500.0;
    //temp = temp/4095.0;
    temp = arrange((uint16_t *)myad,AD_NUM);
    temp = temp*2.27;
    //temp = temp*15.8-2502;
    //temp = temp*2; //temp * 2.99 + 28; //电压变换
    #else
    
    float    temp = 0.0;
    
    //wrtoADS1110();
    //ADC16Result = rdfromADS1110();    

    temp = ADC16Result / 65535.0 * 2048 * 3 + 235;
    #endif
    return (uint16_t)temp;
}

#if 1
// 去掉两个最大值、和两个最小值
uint16_t arrange(uint16_t *a,uint8_t n)
{
    uint8_t i,j;
    uint16_t m;

    //  排序
    for (i=0; i<n-1; i++)
    {
        for (j=i+1; j<n; j++)
        {
            if (a[j]>a[i])
            {
                m=a[i];
                a[i]=a[j];
                a[j]=m;
            }
        }
    }
    
    return ((a[3] + a[4] + a[5] + a[6])/4);
}
#endif

void DeviceGetBatAlarm(STU_BATTERY *bat)
{
  	static uint8_t oldbat = 100;//电池电量显示只降不升
  	bat->Voltage = DeviceGetBatVal();
	
	if(bat->Voltage < 3100)
	{
	  	bat->batPercent = 0;
	}
	else
	{
		bat->batPercent = (bat->Voltage-3100)/10;//电池电压理论3~4.2V，我们弄成3.1~4.1
	}
	if(oldbat < bat->batPercent)
	{
	  	bat->batPercent = oldbat;
	}
	if(bat->batPercent>100)
	{
	  	bat->batPercent = 100;
	}
	if(bat->batPercent<3)
	{
	  	bat->Status = 1;
	}
	else if(bat->batPercent>15)
	{
	  	bat->Status = 0;
	}
	oldbat = bat->batPercent;
}
