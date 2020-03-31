
//电网电压测量
#include "bsp.h"
#include "mydefine.h"

#include <math.h>

//设置 触发电压
extern SPI_HandleTypeDef hspi2;

#define	AD7685_SPI	hspi2

#define		AD7685_CNV_PORT		GPIOB
#define		AD7685_CNV_PIN		GPIO_PIN_12

#define		AD7685_CNV_LOW()		HAL_GPIO_WritePin(AD7685_CNV_PORT, AD7685_CNV_PIN, GPIO_PIN_RESET);
#define		AD7685_CNV_High()		HAL_GPIO_WritePin(AD7685_CNV_PORT, AD7685_CNV_PIN, GPIO_PIN_SET);

#define		AD7685_SDI_PORT		GPIOB
#define		AD7685_SDI_PIN		GPIO_PIN_15

#define		AD7685_SDI_LOW()		HAL_GPIO_WritePin(AD7685_SDI_PORT, AD7685_SDI_PIN, GPIO_PIN_RESET);
#define		AD7685_SDI_High()		HAL_GPIO_WritePin(AD7685_SDI_PORT, AD7685_SDI_PIN, GPIO_PIN_SET);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

void delay_ns(int delayT)
{
	
	
}

void start_chang_adc()
{
	
	AD7685_SDI_High();
	AD7685_CNV_LOW();
//	delay_ns(3);//没要求
	
	delay_ns(15);		//t_ssdicnv ,AD7685_SDI_High等15ns
	AD7685_CNV_High();
	
//	delay_ns(2200); //t_conv 必须等待2.2us才能读取
	
}

void AD7685_adc_read()  //t_acq 1.8us  //1024个数据 19.5312us进一次  ,在 TIM3_IRQHandler(void)中断中运行
{
	u8 buf[2];
	
	AD7685_SDI_LOW();
	delay_ns(3);//没要求
	
	HAL_SPI_Receive(&AD7685_SPI, buf, 2, 100); //读取ADC值
	
//////////////////////////////////////////////////////////////	以上 t_acq
	start_chang_adc(); //开始下一轮转换
}



