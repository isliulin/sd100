
//谐波测量
#include "bsp.h"
#include "mydefine.h"

extern SPI_HandleTypeDef hspi2;

#define	AD7685_SPI	hspi2

#define		AD7685_CS_PORT	GPIOB
#define		AD7685_CS_PIN		GPIO_PIN_12
//#define		AD7685_CS(x)		(((x) == 0) ? HAL_GPIO_WritePin(AD7685_CS_PORT, AD7685_CS_PIN, GPIO_PIN_RESET) : HAL_GPIO_WritePin(AD7685_CS_PORT, AD7685_CS_PIN, GPIO_PIN_SET))
//#define		AD7685_CS(x)		HAL_GPIO_WritePin(AD7685_CS_PORT, AD7685_CS_PIN, x)
#define  AD7685_Enable()   do{unsigned char i;HAL_GPIO_WritePin(AD7685_CS_PORT, AD7685_CS_PIN, GPIO_PIN_RESET);for(i = 0;i < 5;i++);}while(0)
#define  AD7685_Disable()  HAL_GPIO_WritePin(AD7685_CS_PORT, AD7685_CS_PIN, GPIO_PIN_SET);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

void Set_AD7685_DAC(u8 mode,u16 ADC)
{
	CPU_SR_ALLOC();
	u16 out_data;
	u8 sendbuf[2];
	
	CPU_CRITICAL_ENTER(); //关中断
	
	out_data=(mode<<14)+(ADC<<8);
	sendbuf[0]=(out_data>>8);
	sendbuf[1]=out_data;
	
	AD7685_Enable();
	sendbuf[0] = RDID;
	HAL_SPI_Transmit(&hspi2, sendbuf, 1,10);
	AD7685_Disable();

	CPU_CRITICAL_EXIT();//开中断	
}