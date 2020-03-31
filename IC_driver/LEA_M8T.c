
//gps
#include "bsp.h"
#include "mydefine.h"

extern UART_HandleTypeDef huart4;

#define	GPS_UART	huart4


//������ PE1  EXTI1_IRQHandler(void)
#define		M8T_TS_PORT		GPIOE
#define		M8T_TS_PIN		GPIO_PIN_0

#define		M8T_TS_LOW()		HAL_GPIO_WritePin(M8T_TS_PORT, M8T_TS_PIN, GPIO_PIN_RESET);
#define		M8T_TS_High()		HAL_GPIO_WritePin(M8T_TS_PORT, M8T_TS_PIN, GPIO_PIN_SET);

#define HEAD1				0xB5	//181
#define HEAD2				0x62	//98
#define Class_NAV 	0x01
#define ID_TIMEBDS 	0x24

#define Class_TIM 	0x0D

//HEAD1	HEAD2	CLASS	ID	LENTH	PALOAD CK_A	CK_B	//LENTH :	PALOAD�ĳ���
//1			1			1			1		2			lenth		1		1			//���ݴӵ͵���(С��ģʽ)  lenth 01 00  ����Ϊ1


void ublox_checksum(u8 *Buffer,int N,u8 *CK) //148ҳ,class��ʼ��
{
	int i;
	
	CK[0] = 0;
	CK[1] = 0;
	for(i=0;i<N;i++)
	{
		 CK[0] = CK[0] + Buffer[i];
		 CK[1] = CK[1] + CK[0];
	}
}

//GPS Time UBX-NAV-TIMEGPS 
//BeiDou Time UBX-NAV-TIMEBDS 
//GLONASS Time UBX-NAV-TIMEGLO 
//Galileo Time UBX-NAV-TIMEGAL 
//UTC Time UBX-NAV-TIMEUTC
int get_timebds(void)
{
	u8 buf[20],CK[2];
	u16 lenth=0;
	
	uint8_t *udata;	//��ȡ�������ݵ�ַ
	uint16_t ulen,i;	//��ȡ�������ݳ���
	
	buf[0]=HEAD1;
	buf[1]=HEAD2;
	buf[2]=Class_NAV;
	buf[3]=ID_TIMEBDS;
	buf[4]=lenth;	//��λ
	buf[5]=(lenth>>8);//��λ
//�м�Ϊ����	
	ublox_checksum(buf,lenth+4,CK);

	buf[lenth+6]=CK[0];
	buf[lenth+7]=CK[1];
	
	HAL_UART_Transmit_IT(&GPS_UART, buf, lenth+8);
	
	
	if(UART_Receive_s(&GPS_UART,&udata, &ulen,1000) != 2) 
	{
		printf("\r\nû����");
		return 0;
	}
	for(i=0;i<ulen;i++)	printf("%c ",udata[i]);
//	HAL_UART_Transmit_IT(&huart4, udata, ulen); //��ӡ������
	
	return 0;
}

//������ PE1  EXTI1_IRQHandler(void)
void M8T_mmc()
{
	
}








