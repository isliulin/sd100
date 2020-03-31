
//存储
//

#include "bsp.h"
#include "mydefine.h"
#include "MX35.h"

extern SPI_HandleTypeDef hspi1;

#define	MX35_SPI	hspi1

#define		MX35_CS_PORT	GPIOA
#define		MX35_CS_PIN		GPIO_PIN_4
//#define		AD5620_CS(x)		(((x) == 0) ? HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, GPIO_PIN_RESET) : HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, GPIO_PIN_SET))
//#define		AD5620_CS(x)		HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, x)
#define  MX35_Enable()   do{unsigned char i;HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_RESET);for(i = 0;i < 5;i++);}while(0)
#define  MX35_Disable()  HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_SET);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t Timeout);

#define dbg_printf(x,y)  printf(x,y)  //
//#define dbg_printf(x,y)  (char)0u  //

#define  CS_Low()	do{unsigned char i;HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_RESET);for(i = 0;i < 5;i++);}while(0)
#define  CS_High()	HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_SET);

#define  hdl_Spi_SendNbyte(data,Size) HAL_SPI_Transmit(&MX35_SPI, data, Size, 10);
#define  hdl_Spi_RecvNbyte(data,Size) HAL_SPI_Receive(&MX35_SPI, data, Size, 10);
 

void SendByte( uint8_t  byte_value)
{
	uint8_t tdata;
	tdata=byte_value;
	HAL_SPI_Transmit(&MX35_SPI, &tdata, 1,10);
	
}


ReturnMsg CMD_GET_FEATURE( uint8_t addr, uint8_t *StatusReg )
{
   // Chip select go low to start a flash command
	CPU_SR_ALLOC();
	u8 sendbuf[3];
	
	CPU_CRITICAL_ENTER(); //关中断
	
  CS_Low();
	sendbuf[0]=FLASH_CMD_GET_FEATURE;
	sendbuf[1]=addr;
	hdl_Spi_SendNbyte(sendbuf,2);
    //Get Features
	hdl_Spi_RecvNbyte(StatusReg,1);
    // Chip select go high to end a flash command
  CS_High();
	
	CPU_CRITICAL_EXIT();//开中断	
	
  return Flash_Success;
}
ReturnMsg CMD_SET_FEATURE( uint8_t addr, uint8_t value ) 
{
	CPU_SR_ALLOC();
	u8 sendbuf[3];
	
	CPU_CRITICAL_ENTER(); //关中断
	
   // Chip select go low to start a flash command
  CS_Low();
	sendbuf[0]=FLASH_CMD_GET_FEATURE;
	sendbuf[1]=addr;
	sendbuf[2]=value;
	hdl_Spi_SendNbyte(sendbuf,3);
	CS_High();
	
	CPU_CRITICAL_EXIT();//开中断	
	return Flash_Success;
}

int CheckStatus( uint8_t CheckFlag ) 
{
    uint8_t status;
    CMD_GET_FEATURE( 0xc0, &status );
    if( (status & CheckFlag) == CheckFlag )
        return F_BUSY;
    else
        return READY;
}

int WaitFlashReady( void ) //等待完成 1ms 超时时间  80 000 000  对应80000 = 1ms
{
   u32 last_time_tack=0;
   last_time_tack =OS_TS_GET();// mde_stc_GetTick();//获取时间点
	
    while(1)
    {
      if( CheckStatus( SR0_OIP ) == READY ) return READY;
			if((OS_TS_GET()-last_time_tack)>=80000) return TIMEOUT;
    }
} 
ReturnMsg CMD_RESET_OP( void )
{
    CS_Low();
    /* Send reset command */
    SendByte( 0xFF );
    CS_High();
    /* Wait page program finish :等待页面程序完成*/
   if(READY!=WaitFlashReady()) return Flash_F_BUSY;
   else return Flash_Success;
}
 

void GetPageAddr( uint32_t Address,u8 *sent_add )//Plane select： 2Gb的MX35LF2GE4AB才有
{
	#ifdef MX35LF1GE14AB
	if(Address>0xffff) return;
	sent_add[0]=0;
	sent_add[1]=(Address>>8);
	sent_add[2]=Address;
	#endif
}

void GetColAddr( uint16_t Address,u8 *sent_add) //, uint8_t wrap  Wrap[1:0]： 1Gb的MX35LF1GE4AB才有  0-2112 2048+64
{
	char Wrap=0x00;//
	
	sent_add[0]= ((Wrap<<6)&0xc0) + ((Address>>8)&0x0f);
	sent_add[1]= Address; //(Address & 0x0f)
}
ReturnMsg CMD_WREN( void ) //flash 擦写使能打开
{
    // Chip select go low to start a flash command
    CS_Low();
    // Write Enable command = 0x06, Setting Write Enable Latch Bit
    SendByte( FLASH_CMD_WREN );
    // Chip select go high to end a flash command
    CS_High();
    return Flash_Success;
}
ReturnMsg CMD_WRDI( void )
{
    // Chip select go low to start a flash command
    CS_Low();
    // Write Disable command = 0x04, resets Write Enable Latch Bit
    SendByte( FLASH_CMD_WRDI );
    CS_High();
    return Flash_Success;
}
ReturnMsg CMD_RDID( uint16_t *Identification ) //获取芯片ID信息  BY0 0xc2  by1  0x12-1G  0x22-2G
{
    uint16_t temp;
    uint8_t  gDataBuffer[2];
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Chip select go low to start a flash command
    CS_Low();
    // Send command
	gDataBuffer[0]=FLASH_CMD_RDID;
	gDataBuffer[1]=0;
	hdl_Spi_SendNbyte(gDataBuffer,2);
	
	hdl_Spi_RecvNbyte(gDataBuffer,2);
	
    // Chip select go high to end a command
    CS_High();
    // Store identification
    temp =  gDataBuffer[0];
    temp<<=8;
    *Identification = temp| gDataBuffer[1];
	
	printf("\r\nMX35_ID:%x %x",gDataBuffer[0],gDataBuffer[1]);
	
    return Flash_Success;
}


ReturnMsg CMD_READ( uint32_t page_address )
{
	u8 buff[4];
    // Check flash address
    if( page_address> Flash_Page_Size ) return Flash_AddrInvalid;
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Chip select go low to start a flash command
    CS_Low();
    buff[0]=FLASH_CMD_READ;
		GetPageAddr( page_address,&buff[1] );
		hdl_Spi_SendNbyte(buff,4);
    CS_High();
    /* Wait data transfer from array to cache finish */
    if( WaitFlashReady() == READY ) 
    {
        return Flash_Success;
    }
    else
    {
        return Flash_OperationTimeOut;
    }
}

ReturnMsg READ_CACHE_SEQUENTIAL( u8 flag ) //0-结束，最后一页，else-开始
{
	uint8_t buff[4];
	
	CS_Low();
	
	if(flag)	buff[0]=FLASH_CMD_READ_CACHE_SEQUENTIAL;  //31
	else 			buff[0]=FLASH_CMD_READ_CACHE_END;					//3f
	hdl_Spi_SendNbyte(buff,1);
	CS_High();
	
	if( WaitFlashReady() == READY )
	{
		return Flash_Success;
	}
	else
	{
		return Flash_OperationTimeOut;
	}
}

ReturnMsg RANDOM_DATA_READ( u8 *rdata ) //03
{
	uint8_t buff[4];

	CS_Low();	
	buff[0]=FLASH_CMD_READ_CACHE;
	GetColAddr( 0,&buff[1]);  //0-从此页第几个BYE开始读 0-2111
	buff[3]=0;
	hdl_Spi_SendNbyte(buff,4);
	hdl_Spi_RecvNbyte(rdata,2112);	
	CS_High();
	return Flash_Success;
}

ReturnMsg Read_From_Cache_1(uint32_t page_address,u8 *rdata )
{
	if(CMD_READ( page_address )==Flash_OperationTimeOut) return Flash_OperationTimeOut;
	if(RANDOM_DATA_READ( rdata )!= Flash_Success) return Flash_OperationTimeOut;
	
	return Flash_Success;
}

ReturnMsg Read_From_Cache_N(uint32_t page_address,u8 *rdata,u16 N ) //page_address 初始页，读N页
{
	u16 i;

	if(CMD_READ( page_address )==Flash_OperationTimeOut) return Flash_OperationTimeOut;
	
	for(i=N;i!=0;i--)
	{
		if(READ_CACHE_SEQUENTIAL( i-1 )!= Flash_Success) return Flash_OperationTimeOut;

		if(RANDOM_DATA_READ( rdata )!= Flash_Success) return Flash_OperationTimeOut;
		
	}
	
	if(CMD_READ( page_address )==Flash_OperationTimeOut) return Flash_OperationTimeOut;
	if(RANDOM_DATA_READ( rdata )!= Flash_Success) return Flash_OperationTimeOut;
	
	return Flash_Success;
}

uint16_t CMD_PP_LOAD8bit( uint16_t col_address,uint8_t addr_flag, uint8_t *DataBuf, uint16_t byte_size ) //addr_flag 00-2112 01-2048 10-64 11-16   col_address=0
{
	uint8_t buff[4];
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY; 
    //send write enable command
     CMD_WREN();
    // Chip select go low to start a flash command
    CS_Low();
    /* Send program load command */
//    SendByte( FLASH_CMD_PP_LOAD );
	buff[0]=FLASH_CMD_PP_LOAD;
    /* Send flash address */
//    SendColAddr( col_address, addr_flag ); 
	buff[1]=((col_address>>8)|(addr_flag<<4));
	buff[2]=col_address;
	hdl_Spi_SendNbyte(buff,3);
    /* Send data to program */
    if(byte_size>nd_page_bytesize)byte_size=nd_page_bytesize;
    hdl_Spi_SendNbyte(DataBuf,byte_size); 
    // Chip select go high to end a flash command
    CS_High();
    return byte_size;
}
ReturnMsg CMD_PP_RAND_LOAD( uint16_t col_address, uint8_t *DataBuf, u32 Length, uint8_t addr_flag ) //addr_flag 00-2112 01-2048 10-64 11-16   col_address=0
{
//    u32 i;
	uint8_t buff[4];
    // Chip select go low to start a flash command
  CS_Low();
    /* Send program load command */
//    SendByte( FLASH_CMD_PP_RAND_LOAD );
	buff[0]=FLASH_CMD_PP_RAND_LOAD;
    /* Send flash address */
	buff[1]=((col_address>>8)|(addr_flag<<4));
	buff[2]=col_address;
	hdl_Spi_SendNbyte(buff,3);
    /* Send data to program */
//    for( i=0; i<Length; i=i+1 )
//          {
//        SendByte( DataBuf[i] );
//          }
		hdl_Spi_SendNbyte(DataBuf,Length);			
    // Chip select go high to end a flash command
    CS_High();
    return Flash_Success;
}

ReturnMsg CMD_PROGRAM_EXEC( u32 page_address )//程序执行命令 ,让写入数据生效
{
    uint8_t status;
	u8 sendbuf[4];
    // Chip select go low to start a flash command
    CS_Low();
    /* Send program execute command */
//    SendByte( FLASH_CMD_PROGRAM_EXEC ); 
	sendbuf[0]=FLASH_CMD_PROGRAM_EXEC;
    /* Send flash address */
	
	sendbuf[1]=(page_address>>16);
	sendbuf[2]=(page_address>>8);
	sendbuf[3]=(page_address>>0);
	
		hdl_Spi_SendNbyte(sendbuf,4);
	
   // Chip select go high to end a flash command
    CS_High();
 
    /* Wait page program finish */
    if( WaitFlashReady() == READY )//等待完成 1ms 超时时间
    {
        /* Check program result */
        CMD_GET_FEATURE( 0xc0, &status );
        if( (status & SR2_ProgramF_FAIL ) == SR2_ProgramF_FAIL ) 
            return Flash_ProgramF_FAILed;
        else
            return Flash_Success;
    }
    else
    {
        return Flash_OperationTimeOut;
    }
}
/*
 * Function:       CMD_BE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE instruction is for erasing the data
 * Return Message: Flash_AddrInvalid, Flash_F_BUSY, Flash_Success,
 *                 Flash_OperationTimeOut
 */
ReturnMsg CMD_BE( u32 page_address ) //擦除flash
{
    uint8_t status,sendbuf[4];
    // Check flash address
    if( page_address > Flash_Page_Size ) return Flash_AddrInvalid;
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Setting Write Enable Latch bit
    CMD_WREN();
    // Chip select go low to start a flash command
    CS_Low();
    //Write Block Erase command
//    SendByte( FLASH_CMD_BE ); 
	sendbuf[0]=FLASH_CMD_BE;
//    SendRowAddr( flash_address>>12); //发送块地址信息
	
	sendbuf[1]=(page_address>>16);
		sendbuf[2]=(page_address>>8);
		sendbuf[3]=(page_address>>0);
		hdl_Spi_SendNbyte(sendbuf,4);
	
    // Chip select go high to end a flash command
    CS_High();
      /* Wait page program finish */
    if( WaitFlashReady() == READY ) //等待完成 1ms超时时间
    {
        /* Check program result */
        CMD_GET_FEATURE( 0xc0, &status );
        if( (status & SR2_EraseF_FAIL ) == SR2_EraseF_FAIL ) 
            return Flash_EraseF_FAILed;
        else
            return Flash_Success;
    }
    else
    {
        return Flash_OperationTimeOut;
    }
}

