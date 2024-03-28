#ifndef _MAIN_H
#define _MAIN_H

#include "STC32G.h"
#include "stdio.h"
#include "intrins.h"
#include "string.h"
#include "stdlib.h"

#ifdef __cplusplus
 extern "C" {
#endif


//#define ZGC_DOSE

#define P2P_NH         // ��Ե���� --   ��ʱֻ֧�ֵ�GM��̽ͷ

//��������汾��
#define VERSION1 3
#define VERSION2 0
#define VERSION3 4


typedef     unsigned char    uint8_t;
typedef     unsigned short    uint16_t;
typedef     unsigned long    uint32_t;
typedef     unsigned int    uint;    
   
typedef     unsigned char    BOOL;
#define     TRUE    1
#define     FALSE   0

#define     true    1
#define     false   0

typedef     unsigned char    BYTE;
typedef     unsigned short    WORD;
typedef     unsigned long    DWORD;

typedef     unsigned char    u8;
typedef     unsigned short    u16;
typedef     unsigned long    u32;   

#define bool BYTE

#define alt_u8   BYTE
#define alt_u16 WORD
#define alt_u32 DWORD      
     
#define MAIN_Fosc     6000000UL   //11059200UL      // 11.0592M
//#define MAIN_Fosc        35000000UL    // 35M

#define SysTick     9216   // 10ms    // ��/��, ϵͳ�δ�Ƶ��, ��4000~16000֮��

#define Timer0_Reload   (65536UL - SysTick)     //Timer 0 �ж�Ƶ��

#define FIX_COUNT 11  // ��������
#define NH_COUNT  8   // ��ϵ���

   
#define SENSOR_LONGPOLE   0x2a                   //����̽ͷ
#define POWER_OFF_TIME   (2*60*100)
#define LOW_DIBENDI        0   
#define LOW_GAOBENDI    100
#define HIGH_DIBENDI    0   
#define HIGH_GAOBENDI    50   

#define MIB_CST_DOSERATE_THRESHOLD_ALARM    2    
#define MIB_CST_DOSERATE_THRESHOLD_WARNING    2
   
#define DOSERATE_PRE_ALARM_BIT  (1<<0)            //������Ԥ��λ
#define DOSERATE_ALARM_BIT      (1<<1)            //�����ʱ���λ
#define DOSE_PRE_ALARM_BIT      (1<<2)            //��������Ԥ��λ
#define DOSE_ALARM_BIT          (1<<3)            //������������λ
#define BATTARY_LOW_BIT         (1<<4)            //��������λ
#define LOW_BACKGROND_BIT       (1<<5)            //�ͱ��ױ���λ
#define HIGH_BACKGROND_BIT      (1<<6)            //�߱��ױ���λ

#define Uart485_EN(x) (x)?(P4 |= (1<<6)):(P4 &= ~(1<<6))      // RS485
#define Light_M(x)     (x)?(P5 |= (1<<2)):(P5 &= ~(1<<2))       // Light

#if 0
#define BT_PWON     P5OUT = BIT6
#define BT_PWOFF    P5OUT &= (~BIT6)
#define BT_WAKEUP   P5OUT = BIT2
#define BT_SLEEP    P5OUT &= (~BIT2)
#define BT_CONFIG   P5OUT |= BIT3
#define BT_WORK     P5OUT &= (~BIT3)
#endif
 
#define BLUE_PWON()     P4 |= (1<<1)
#define BLUE_PWOFF()    P4 &= ~(1<<1)
#define BLUE_SLEEP()    P4 &= ~(1<<3)       //���ߣ��͹���ģʽ��ֻ�ܽ�������
#define BLUE_WAKEUP()   P4 |= (1<<3)        //���ѣ��ߵ�ƽ����ʱ10ms���ܷ�������
#define BLUE_CFG_MODE() P4 |= (1<<2)        //��������ģʽ
#define BLUE_WORK_MODE()    P4 &= ~(1<<2)   //��������ģʽ

#pragma pack(2)
//ϵͳ���ò���
typedef struct
{
      float  Hv;      // 0x01+��ѹֵ
    float  Z1;      //0x02+�������ֵ1
    float  Ct;      // 0x03+����ʱ��
    float  Hd;      //0x04+��ѹ��
    float  Z2;      // 0x05+�������ֵ2
    float  Z3;      //0x06+�������ֵ3
    float  S1;      // 0x07+̽��������ϵ��1
    float  S2;      //0x08+̽��������ϵ��2
    float  Cr;      // 0x09+У׼����
    uint8_t  Hn;    //0x0A+���ú��غ�
    float  S3;      //0x0B ̽��������ϵ��3
    float  Z4;      //0x0C �������ֵ4

    float PingHuaShiJian;   //ƽ��ʱ��

    float DiYaCanshuA;      //������ͨ��У׼����A             LP
    float DiYaCanshuB;      // ������ͨ��У׼����B           LP
    float DiYaCanshuC;      // ������ͨ��У׼����C             LP
    float GaoYaCanshuA;     //������ͨ��У׼����A     LP
    float GaoYaCanshuB;     // ������ͨ��У׼����B     LP
    float GaoYaCanshuC;     // ������ͨ��У׼����C     LP

//    float BCanshuA;         //��У׼����A         LP
//    float BCanshuB;         //��У׼����B        LP
//    float BCanshuC;         //��У׼����C        LP

    
}SYS_PRAM;

//������Ϣ
typedef struct
{
      float  A1;          // 0x01+һ��������ֵ(��)
    float  A2;          //0x02+����������ֵ(��)
    float  Al;          // 0x03+�ͱ��ױ�����ֵ(��)
    float  Ah;          //0x04+�߱��ױ�����ֵ(��)
    float  B1;          // 0x05+һ��������ֵ(��)
    float  B2;          //0x06+����������ֵ(��)
    float  Bl;          // 0x07+�ͱ��ױ�����ֵ(��)
    float  Bh;          //0x08+�߱��ױ�����ֵ(��)
    float  Ac;          // 0x09+alphy���ر���ֵ
    float  Bc;          //0x0A+beta���ر���ֵ
    float  DoseRatePreAlarm;    //0x0B Y������һ������(uSv/h)
    float  DoseRateAlarm;       //0x0C Y�����ʶ�������(uSv/h)
    float  Y3;                  //0x0D Y��������������(uSv/h)
    float  DosePreAlarm;        //0x0E �ۼƼ���Ԥ����ֵ
    float  DoseAlarm;           //0x0F �ۼƼ���������ֵ
    uint8_t   Zu;               //0x10 �ڲ��ۼƼ���������
}SYS_ALARM;

//�����ṹ��    LP
typedef struct
{
    float  Dose_B;  uint8_t AjSt;           //0x01+���ڵ���//���ۻ�����
    float  MaxDoseRate_B;  uint8_t HvSt;    //0x02+��ѹ����//����������
    float  C1;  uint8_t C1St;               //0x03+����ֵ1+����״̬��1B������������//C1 ��ͨ�������� LP
    float  C2;  uint8_t C2St;               //0x04+����ֵ2+����״̬��1B�����¼�����//C2 ��ͨ�������� LP
    float  C3;  uint8_t C3St;               // 0x05+����ֵ3+����״̬��1B��(��+��)//�¼�����
    float  P1;  uint8_t P1St;               //0x06+ƽ������ֵ1+����״̬��1B������������//P1��ͨ��ԭʼ���� LP
    float  P2;  uint8_t P2St;               //0x07+ƽ������ֵ2+����״̬��1B������������//P2��ͨ��ԭʼ���� LP
    float  P3;  uint8_t P3St;               //0x08+ƽ������ֵ3+����״̬��1B������������//�¼���
    float  DoseRate;  uint8_t DRSt;         // 0x09+������+����״̬��1B��uSv/h   LP
    float  Dose;  uint8_t DoSt;             // 0x0A+�ۼƼ���+����״̬��1B��uSv   LP
    float  MaxDoseRate;  uint8_t MRSt;        //��������uSv/h   LP 

}STU_DOSERATE;

#pragma pack()

typedef struct
{
    uint8_t SensorType;    //̽ͷ����
    uint8_t res;
    uint8_t VerSion1;        //��汾
    uint8_t VerSion2;        //С�汾
    
    uint8_t VerSion3;        //SVN��    
    uint8_t res2;
    SYS_PRAM s_SysParam;

    SYS_ALARM s_Alarm;

    float Fix[FIX_COUNT];   // С�˴洢������ͨ�ţ�ʹ�õ�ʱ��ת��һ��
    
    uint16_t ParaCheckSum;//����У���
}LP_PARAM;

typedef struct
{
      LP_PARAM    stParam;
    uint8_t isSleep;
    uint8_t isCanReadSensor;
    STU_DOSERATE s_DoseMSG;
    uint16_t DoseRatePreAlarmCnt;           //������Ԥ������������2�α����ܱ���
    uint16_t DoseRateAlarmCnt;              //�����ʱ�������������2�α����ܱ���
    uint32_t NoUartTime;                    //�޴���ͨ��ʱ��
    uint32_t LChannelNoCountTime;            //��ͨ���޼���ʱ��
    uint32_t HChannelNoCountTime;            //��ͨ���޼���ʱ��
    uint8_t     LowChanneloff;                    //��ͨ���Ƿ�رգ��رպ�Ͳ��Ʊ���
    //uint16_t testtime;
}LP_SYSTEM_STTAE;

//��ص����ṹ��
typedef struct
{
    uint16_t Voltage;            //��ص�ѹֵmv       LP
    uint8_t  batPercent;        //��ذٷֱ�%        LP
    uint8_t  Status;            //0��������1����ص�ѹ��
}STU_BATTERY;


typedef struct
{
    float cps;
    float dr;
}NH_PARAM;


extern LP_SYSTEM_STTAE SysRunState;

void Error();
void InitParam();

void delay_ms(WORD ms);
void delay_us(BYTE us);
void TimerTask();
void BleHnd();
void Light_Run(u16 dt);


#endif

