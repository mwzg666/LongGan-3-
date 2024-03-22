#include "main.h"
#include "mcp4725.h"
#include "i2c.h"

//========================================================================
// 函数名称: void MCP4725_OutVol(BYTE addr,char *voltage)
// 函数功能: 转换输出为电压值
// 入口参数: @addr：I2C地址；@*voltage：要转换的数据指针
// 函数返回: 无
// 当前版本: VER1.0
// 修改日期: 2023.5.5
// 当前作者:
// 其他备注: 
//========================================================================

void MCP4725_OutVol(BYTE addr,unsigned int voltage)
{
	DWORD uiVolTemp;
    unsigned int vol_tmp;
	BYTE out_data = 0;
	vol_tmp = voltage;
	if(vol_tmp >= MCP4725_REF_VOL)
	{
		//printf("input voltage > ref voltage\r\n");
		return;
	}

    uiVolTemp = vol_tmp;
	uiVolTemp = uiVolTemp*4096l/MCP4725_REF_VOL;
	I2C_Start(1);
	out_data = addr<<1;
	I2C_Write_Byte(1);
	I2C_ReadAck(1);
	out_data = ((uiVolTemp>>8)&0xf)|MCP4725_PD_MODE;   // 高8位
    //out_data = uiVolTemp & 0xff;       // 低8位
	I2C_Write_Byte(1);
	I2C_ReadAck(1);
    //out_data = ((uiVolTemp>>8)&0xf)|MCP4725_PD_MODE;   // 高8位
	out_data = uiVolTemp & 0xff;       // 低8位
	I2C_Write_Byte(1);
	I2C_ReadAck(1);
	I2C_Stop(1);
}

#if 0
unsigned int MCP4725_Read(unsigned char addr)
{
	unsigned int uiTemp1,uiTemp2,dat;

	I2C_Start();
	I2C_Write_Byte((addr<<1) | 1);
	I2C_Read_Byte(I2C_ACK);
	uiTemp1 = I2C_Read_Byte(I2C_ACK);
	uiTemp2 = I2C_Read_Byte(I2C_NACK);
	I2C_Stop();
	dat = ((uiTemp1&0xf)<<4 |(uiTemp2>>4) ) | (uiTemp1<<4);	
	return dat;
}
#endif

