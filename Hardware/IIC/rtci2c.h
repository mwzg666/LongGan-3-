#ifndef RTCI2C_H_
#define RTCI2C_H_

#define RTCI2C_SDA_MASK 1//定义IIC 的SDA  IO口
#define RTCI2C_SCL_MASK 2//定义IIC 的SCL  IO口

#define RTCI2C_HIGH 1
#define RTCI2C_LOW  0

#define RTCI2C_ACK 0
#define RTCI2C_NACK RTCI2C_SDA_MASK

#include "main.h"

//函数声明
//void RTCI2C_Init(void);
//void RTCI2C_Start();
//void RTCI2C_Stop();
//unsigned char RTCI2C_Write_Byte(char out_data);
//unsigned char RTCI2C_Read_Byte(char ack);


#endif
