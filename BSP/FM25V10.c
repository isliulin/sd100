#include "stm32l4xx_hal.h"
#include "FM25V10.h"
#include <os.h>
#include "bsp.h"
#include "global.h" 
//#include <os_app_hooks.h>


#define  FM25_Enable()   do{unsigned char i;HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);for(i = 0;i < 5;i++);}while(0)
#define  FM25_Disable()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

extern SPI_HandleTypeDef hspi2;
///////////////////////////////////////FM25///////////////////////////////////////

// /**
//  * @brief  Reads a byte from the SPI Flash.
//  *   This function must be used only if the Start_Read_Sequence
//  *   function has been previously called.
//  * @param  None
//  * @retval : Byte Read from the SPI Flash.
//  */
void ReadFM25DeviceID(unsigned char *buf)
{
	CPU_SR_ALLOC();
	uint8_t sendbuf[1];
	CPU_CRITICAL_ENTER(); //关中断

	FM25_Enable();
	sendbuf[0] = RDID;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	HAL_SPI_Receive(&hspi2, buf, 9,10);	
	FM25_Disable();

	CPU_CRITICAL_EXIT();//开中断	
}
/**
 Set Write Enable Latch
The WREN command must be issued prior to any write operation.
 Completing any write operation will automatically
 clear the write-enable latch and prevent further
writes without another WREN command.
*/
void SetFM25WriteEnable(void)
{
	uint8_t sendbuf[4];
	FM25_Enable();
	sendbuf[0] = WREN;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	FM25_Disable();
}
/**
 clearing the Write Enable Latch
*/
void ClearFM25WriteEnable(void)
{
	uint8_t sendbuf[4];
	FM25_Enable();
	sendbuf[0] = WRDI;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	FM25_Disable();
}

/*
Write Operation
All writes to the memory array begin with a WREN op-code. 
The next op-code is the WRITE instruction. 
This op-code is followed by a three-byte address value, 
which specifies the 17-bit address of the first data byte of the write operation.
Subsequent bytes are data and they are written sequentially. 
Addresses are incremented internally as long as the bus master continues to issue clocks.
If the last address of 1FFFFh is reached, the counter will roll over to 00000h.
Data is written MSB first.
*/
void WriteFM25Operation(unsigned int addr,unsigned char *databuf,unsigned int len)
{	
	uint32_t i;
	unsigned char sendbuf[4];
	CPU_SR_ALLOC(); 
	CPU_CRITICAL_ENTER();
	SetFM25WriteEnable();
  FM25_Enable();
	sendbuf[0] = WRITE;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)(addr >>16);
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)(addr >>8);
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)addr;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);	 
	for(i = 0;i < len;i++)
	 HAL_SPI_Transmit(&hspi2, &databuf[i], 1,10);	
	FM25_Disable();
	ClearFM25WriteEnable();   //   写禁能
	CPU_CRITICAL_EXIT();	

}
/*
Read Operation
After the falling edge of /S, the bus master can issue a READ op-code. 
Following this instruction is a three-byte address value (A16-A0),
specifying the address of the first data byte of the read operation.
Addresses are incremented internally as long as the bus master continues to issue clocks. 
If the last address of 1FFFFh is reached, the counter will roll over to 00000h. Data is read MSB first.
*/
void ReadFM25Operation(unsigned int addr,unsigned char *databuf,unsigned int len)
{

	unsigned char sendbuf[4];
	CPU_SR_ALLOC(); 
	CPU_CRITICAL_ENTER();
	FM25_Enable();
	sendbuf[0] = READ;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)(addr >>16);
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)(addr >>8);
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	sendbuf[0] = (unsigned char)addr;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	HAL_SPI_Receive(&hspi2, databuf, len,10);	
	FM25_Disable();
	CPU_CRITICAL_EXIT();	
}

//#define STAA_add		0

//#define flash_save(ADD,len)	WriteFM25Operation(ADD-STAA+STAA_add,ADD,len)
int flash_read(uint8_t *ADD,uint8_t *buf,int len)
{
//	int i;
		
	ReadFM25Operation(ADD-sys_buf_sram+sys_buf_flash,buf,len);
//	printf("\r\n读取数据:");
//	if(len<=10)
//	for(i=0;i<len;i++) printf("%d ",ADD[i]);
//	else
//	{
//		for(i=0;i<5;i++) printf("%d ",buf[i]);printf("...");
//		for(i=len-5;i<len;i++) printf("%d ",buf[i]);
//	}
	
	return 0;
}
int flash_save(uint8_t *ADD,uint8_t *buf,int len)
{
//	int i;
	
//	printf("\r\n存储数据:");
//	for(i=0;i<len;i++) printf("%d ",buf[i]);
	WriteFM25Operation(ADD-sys_buf_sram+sys_buf_flash,buf,len);
	
//	ReadFM25Operation(ADD-sys_buf_sram+sys_buf_flash,buf,len);
//	printf("\r\n验证数据:");
//	for(i=0;i<len;i++) printf("%02X ",buf[i]);
	
	return 0;
}


