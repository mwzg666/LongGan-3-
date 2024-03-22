#ifndef FLASH_H_
#define FLASH_H_

//===============================变量定义=============================
#define FLASH_PARA_START_ADDR   0x1800          //INFO A start address,used to storage para
#define FLASH_PARA_END_ADDR     0x19ff          //INFO A end address,used to storage para

#define FLASH_DATA_START_ADDR 0x010000          //累计剂量存储地址
#define FLASH_DATA_END_ADDR     0x016400     	//从10000开始的50个block，总工25600字节，可存储6400条数据,保证存储8年，最大80年，7.1小时循环一次

#define BOOTLOADER_START_ADDR   0x1A400         // 4K for bootloader

#define FLASH_UPDATE_ADDR       0xE9FE

#define APP_CORRECT_ADDR	    0x4400		    //应用程序有效与否地址


//===============================结构体定义===========================




//===============================变量声明=============================



//===============================函数声明=============================
void checkApp(void);
void EEPROM_read(u32 EE_address,u8 *DataAddress,u16 length);
u8 EEPROM_write(u32 EE_address,u8 *DataAddress,u16 length);
void EEPROM_SectorErase(u32 EE_address);


#endif