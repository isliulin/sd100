
//电池电压和取电电压测量
#include "bsp.h"
#include "mydefine.h"

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

//typedef enum
//{
//  HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03
//} HAL_StatusTypeDef;

#define ADS_IIC	hi2c2

#define ADS1110A0		0x90	//CVD 电容电池电压监测范围：0～3.0V 
#define ADS1110A1		0x92	//线缆电源模块电压监测范围：0～5.5V 
#define ADS1110A2		0x94
#define ADS1110A3		0x96
#define ADS1110A4		0x98
#define ADS1110A5		0x9a
#define ADS1110A6		0x9c
#define ADS1110A7		0x9e

//Bit 4: SC  When SC is 1, the ADS1110 is in single conversion mode; when SC is 0, the ADS1110 is in continuous conversion mode
//Bits 3?2: DR  1(1) 1(1) 15SPS(1)  32768
//Bits 1?0: PGA  0(1) 0(1) 1(1);  0 1 2; 1 0 4;   1 1 8 ;
u16	ADC_A0,ADC_A1,VolA0,VolA1;
u8 gain;

void ADS1110_configure_gain(u8 PGA)// u8 SC,u8 DR,
{
	CPU_SR_ALLOC();
	HAL_StatusTypeDef HAL_state;
	u8 configuration_registe=0x80;
	
	if(PGA==0) ;
	else if(PGA==2) configuration_registe |= 0x01;
	else if(PGA==4) configuration_registe |= 0x02;
	else if(PGA==8) configuration_registe |= 0x03;
	
	CPU_CRITICAL_ENTER(); //关中断
	HAL_state=HAL_I2C_Master_Transmit(&ADS_IIC, ADS1110A0,&configuration_registe, 1, 100);
	HAL_state=HAL_I2C_Master_Transmit(&ADS_IIC, ADS1110A1,&configuration_registe, 1, 100);
	CPU_CRITICAL_EXIT();//开中断	
}

//Output Code == 32768(DATA RATE==15SPS) * PGA * (Vin+ - Vin-)/2.048V
void ADS1110_read_the_output()// u8 SC,u8 DR,
{
	CPU_SR_ALLOC();
	HAL_StatusTypeDef HAL_state;
	u8 iic_d[3];
	
	CPU_CRITICAL_ENTER(); //关中断
	HAL_state=HAL_I2C_Master_Receive(&ADS_IIC, ADS1110A0, iic_d, 2, 100);
	ADC_A0=(iic_d[0]<<8)+iic_d[1];
	HAL_state=HAL_I2C_Master_Receive(&ADS_IIC, ADS1110A1, iic_d, 2, 100);
	ADC_A1=(iic_d[0]<<8)+iic_d[1];
	CPU_CRITICAL_EXIT();//开中断	
	
	VolA0=ADC_A0*2048/gain/32768;//mV
	VolA1=ADC_A1*2048/gain/32768;//mV
}
//cat > /home/shiyanlou/test.sh << EOF
//$ sudo apt-get update
//$ sudo apt-get install locate
//history | cut -c 8- | cut -d ' ' -f 1 | uniq





