#include "MX35.h"
#include "bsp.h"
#include "mydefine.h"

//底层驱动接口

extern SPI_HandleTypeDef hspi1;

#define	MX35_SPI	hspi1

#define		MX35_CS_PORT	GPIOD
#define		MX35_CS_PIN		GPIO_PIN_8
//#define		AD5620_CS(x)		(((x) == 0) ? HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, GPIO_PIN_RESET) : HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, GPIO_PIN_SET))
//#define		AD5620_CS(x)		HAL_GPIO_WritePin(AD5620_CS_PORT, AD5620_CS_PIN, x)
#define  MX35_Enable()   do{unsigned char i;HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_RESET);for(i = 0;i < 5;i++);}while(0)
#define  MX35_Disable()  HAL_GPIO_WritePin(MX35_CS_PORT, MX35_CS_PIN, GPIO_PIN_SET);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t Timeout);

//#define dbg_printf(x,y)  printf(x,y)  //
#define dbg_printf(x,y)  (char)0u  //

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

//int hdl_Spi_SendNbyte(uint8_t *pData,uint16_t Size)
//{
//	HAL_SPI_Transmit(&MX35_SPI, pData, Size, 10);
//}

//void CS_Low()
//{
//    hdl_cs(0);
//}
//void CS_High()
//{
//    hdl_cs(1);
//}


//uint8_t SendByte( uint8_t  byte_value)
//{
//   return hdl_Spi_Sendbyte(byte_value)&0xff;
//}

/*
 * Function:       Get_Feature
 * Arguments:      addr, Set Feature Address 
 *                       StatusReg, 8 bit buffer to store Feature register value
 * Description:    Check Features Settings.              
 * Return Message: Flash_Success
  */
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
 
/*
 * Function:       Set_Feature
 * Arguments:      addr, Set Feature Address
 *                       value, 8 bit Feature register value to updata
 * Description:    Write Features Settings.           
 * Return Message: Flash_Success
 */
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

/*
 * Function:     CheckStatus
 * Arguments:    CheckFlag -> the status bit to check
 * Return Value: READY, F_BUSY
 * Description:  Check status register bit 7 ~ bit 0
 */
int CheckStatus( uint8_t CheckFlag ) 
{
    uint8_t status;
    CMD_GET_FEATURE( 0xc0, &status );
    if( (status & CheckFlag) == CheckFlag )
        return F_BUSY;
    else
        return READY;
}

/*
 * Function:       WaitFlashReady
 * Arguments:      None             
 * Description:    If flash is ready return READY.             
 *                 If flash is time-out return TIMEOUT.
 * Return Message: READY, TIMEOUT
 */
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
//int WaitFlashReady( void ) //等待完成 1ms 超时时间
//{
//   uint16_t last_time_tack=0;
//   uint16_t tick_ret=0;
//   uint16_t cur_time_tack =0;
//   last_time_tack = mde_stc_GetTick();//获取时间点
//    while(1)
//    {
//        if( CheckStatus( SR0_OIP ) == READY ) return READY;
//        cur_time_tack = mde_stc_GetTick();//获取时间点
//        tick_ret=cur_time_tack >= last_time_tack ? (cur_time_tack - last_time_tack)\
//                        : (Sysms_time_MAX + cur_time_tack - last_time_tack);
//        if(tick_ret>3)return TIMEOUT;
//    }
//}

 
/*
 * Function:     Reset_OP
 * Arguments:    None
 * Return Value: Flash_Success
 * Description:  The reset command FFh resets the read/program/erase operation            
 */
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
 
  
/*
 * Function:     SendColAddr
 * Arguments:    Address, 16 bit col address
 *                    wrap, wrap address bits define the four wrap length
 *                    io_mode, I/O mode to transfer address
 * Return Value: None.
 * Description:  Send col address
 */
 
//void SendColAddr( uint16_t Address, uint8_t wrap ) //发送  flash 列地址信息
//{
//    // Col_A11:A0: 页data寄存器地址
//	u8 sendbuf[3];
//	
////    uint16_t ColAddr;
////	
////		ColAddr = Address & 0xFFFF;//12bit
////	
////    ColAddr>>=8;
////    wrap<<=4;
////    /* Send 16 bit address data */
////    SendByte( (ColAddr & BYTE_MASK)|wrap);
////    SendByte( (Address & BYTE_MASK) );
//	
//	sendbuf[0]=((Address>>8)|(wrap<<4));
//	sendbuf[1]=Address;
//	hdl_Spi_SendNbyte(sendbuf,2);
//}
 
/*
 * Function:     SendRowAddr
 * Arguments:    Address, 32 bit Row address
 *                    io_mode, I/O mode to transfer address
 * Return Value: None.
 * Description:  Send Row address
 */
//void SendRowAddr( uint32_t Address ) //发送行地址 flash
//{
//    /* Send 24 bit address data */
//    // Row_A15:A6 : 块地址
//    // Row_A5:A0 : 页地址
//	u8 sendbuf[3];
////		uint32_t temp;
////		temp = Address>>16;
////	
////    SendByte(temp & BYTE_MASK);
////	
////    temp = Address>>8;
////    SendByte(temp & BYTE_MASK);
////    SendByte(Address & BYTE_MASK);
//	sendbuf[0]=(Address>>16);
//	sendbuf[1]=(Address>>8);
//	sendbuf[2]=Address;
//	hdl_Spi_SendNbyte(sendbuf,3);
//}
 
/*
 * Function:       CMD_WREN
 * Arguments:      None
 * Description:    The WREN instruction is for setting rite Enable Latch
 *                 (WEL) bit.
 * Return Message: Flash_Success
 */
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
 
/*
 * Function:       CMD_WRDI
 * Arguments:      None
 * Description:    The WRDI instruction is to reset
 *                 Write Enable Latch (WEL) bit.
 * Return Message: Flash_Success
 */
ReturnMsg CMD_WRDI( void )
{
    // Chip select go low to start a flash command
    CS_Low();
    // Write Disable command = 0x04, resets Write Enable Latch Bit
    SendByte( FLASH_CMD_WRDI );
    CS_High();
    return Flash_Success;
}
 
 
/*
 * Function:       CMD_RDID
 * Arguments:      Identification, 16 bit buffer to store id
 * Description:    The RDID instruction is to read the manufacturer ID
 *                 of 1-byte and followed by Device ID of 1-byte.
 * Return Message: Flash_F_BUSY,Flash_Success
 */
ReturnMsg CMD_RDID( uint16_t *Identification ) //获取芯片ID信息
{
    uint16_t temp;
    uint8_t  gDataBuffer[2];
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Chip select go low to start a flash command
    CS_Low();
    // Send command
//   SendByte( FLASH_CMD_RDID);
//    SendByte( 0); //哑数据
	gDataBuffer[0]=FLASH_CMD_RDID;
	gDataBuffer[1]=0;
	hdl_Spi_SendNbyte(gDataBuffer,2);
    // Get manufacturer identification, device identification
//    gDataBuffer[0] = SendByte( 0 );
//    gDataBuffer[1] = SendByte( 0 );
	hdl_Spi_RecvNbyte(gDataBuffer,2);
	
    // Chip select go high to end a command
    CS_High();
    // Store identification
    temp =  gDataBuffer[0];
    temp<<=8;
    *Identification = temp| gDataBuffer[1];
    return Flash_Success;
}
 
/*
 * Read Command
 */
/*
 * Function:       CMD_READ
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The READ instruction is for reading data from array to cache.
 * Return Message: Flash_AddrInvalid, Flash_F_BUSY, Flash_Success,Flash_OperationTimeOut
 */
ReturnMsg CMD_READ( uint32_t flash_address )
{
	u8 sendbuf[4];
    // Check flash address
    if( flash_address> FlashSize ) return Flash_AddrInvalid;
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Chip select go low to start a flash command
    CS_Low();
    // Write READ command and address
//    SendByte( FLASH_CMD_READ );//13H
		sendbuf[0]=FLASH_CMD_READ;
	
//    SendRowAddr( flash_address>>12 );
	
		sendbuf[1]=(flash_address>>28);
		sendbuf[2]=(flash_address>>20);
		sendbuf[3]=(flash_address>>12);
		hdl_Spi_SendNbyte(sendbuf,4);
	
    // Chip select go high to end a flash command
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
 
/*
 * Random Data Read Command
 */
/*
 * Function:       CMD_READCache
 * Arguments:      col_address, 16 bit flash memory address
 *                     DataBuf, Data buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 *                       addr_flag, define wrap bit and Plane select bit (only for 2Gb and 4Gb)
 * Description:    The READCache instruction is for reading data out from cache on SO.
 * Return Message: Flash_Success
 */
uint16_t CMD_READ_CACHE32bit( uint16_t col_address, uint8_t addr_flag, uint32_t * DataBuf, uint16_t word_size) 
{
    uint32_t index;
    uint32_t dat_temp =0;
    uint8_t buff[4];
	
    // Chip select go low to start a flash command
    CS_Low();
    // Write READ Cache command and address
//    SendByte( FLASH_CMD_READ_CACHE );
//    SendByte( 0); 
	buff[0]=FLASH_CMD_READ_CACHE;
	buff[1]=0;
	
//    SendColAddr( col_address, addr_flag ); 
	
	buff[2]=((col_address>>8)|(addr_flag<<4));
	buff[3]=col_address;
	hdl_Spi_SendNbyte(buff,4);
	
    // Set a loop to read data into buffer
    if(word_size>nd_page_wordsize)word_size =nd_page_wordsize;
    for( index=0; index < word_size; index++ )
    {
        // Read data one byte at a time
//        buff[0] = SendByte( 0 );
//        buff[1] = SendByte( 0 );
//        buff[2] = SendByte( 0 );
//        buff[3] = SendByte( 0 );
			
			hdl_Spi_RecvNbyte(buff,4);
        dat_temp = (uint32_t)buff[0]<<24;
        dat_temp |= (uint32_t)buff[1]<<16;
        dat_temp |= (uint32_t)buff[2]<<8;
        dat_temp |= (uint32_t)buff[3];
        *DataBuf = dat_temp;
        DataBuf+=1;//2;
    }
    // Chip select go high to end a flash command
    CS_High();
    return word_size;
}
 
uint16_t CMD_READ_CACHE8bit( uint16_t col_address, uint8_t addr_flag, uint8_t * DataBuf, uint16_t byte_size  ) 
{
    uint16_t index;
	uint8_t buff[4];
    // Chip select go low to start a flash command
    CS_Low();
	
    // Write READ Cache command and address
//    SendByte( FLASH_CMD_READ_CACHE ); 
//    SendByte( 0); 
	buff[0]=FLASH_CMD_READ_CACHE;
	buff[1]=0;
	
//    SendColAddr( col_address, addr_flag ); 
	buff[2]=((col_address>>8)|(addr_flag<<4));
	buff[3]=col_address;
	hdl_Spi_SendNbyte(buff,4);
	
    if(byte_size>nd_page_bytesize)byte_size =nd_page_bytesize;
    // Set a loop to read data into buffer
    for( index=0; index < byte_size; index++ )
    {
        // Read data one byte at a time
//        DataBuf[index] = SendByte( 0 );
			hdl_Spi_RecvNbyte(&DataBuf[index],1);
    }
    // Chip select go high to end a flash command
    CS_High();
    return byte_size;
}
 
/*
 * Page Read Cache Sequential Command
 */
/*
 * Function:       CMD_READ_CACHE_SEQUENTIAL
 * Arguments:      None.
 * Description:    The READCacheSequential instruction is for throughput enhancement 
 *                 by using the internal cache buffer.
 * Return Message: Flash_F_BUSY, Flash_Success,Flash_OperationTimeOut
 */
ReturnMsg CMD_READ_CACHE_SEQUENTIAL( void )
{
 
    // Chip select go low to start a flash command
    CS_Low();
    // Write READ Cache Sequential command 
    SendByte( FLASH_CMD_READ_CACHE_SEQUENTIAL );
    // Chip select go high to end a flash command
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
 
/*
 * Page Read Cache End Command
 */
/*
 * Function:       CMD_READ_CACHE_END
 * Arguments:      None.
 * Description:    The READCacheEnd instruction is for ending reading
 *                 data from cache buffer.
 * Return Message: Flash_F_BUSY, Flash_Success,Flash_OperationTimeOut
 */
ReturnMsg CMD_READ_CACHE_END( void )
{
 
    // Chip select go low to start a flash command
    CS_Low();
    // Write READ Cache End command 
    SendByte( FLASH_CMD_READ_CACHE_END );
    // Chip select go high to end a flash command
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
 
/*
 * Program load Command
 */
/*
 * Function:       CMD_Program_Load
 * Arguments:      col_address, 16 bit col address
 *                 DataBuf, buffer of source data to program
 *                 byte_length, byte length of data to programm
 *                 addr_flag, define Plane select bit (only for 2Gb and 4Gb)
 * Description:    load program data with cache reset first
 * Return Message: Flash_F_BUSY, Flash_Success,
 */
uint16_t CMD_PP_LOAD32bit( uint16_t col_address,uint8_t addr_flag, uint32_t *DataBuf, uint16_t word_size ) //32bit页编程模式
{
    uint32_t dat_temp =0;
    uint8_t buff[4];
    uint32_t index =0;
 
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY; 
    //send write enable command
     CMD_WREN();//flash 擦写使能打开
    // Chip select go low to start a flash command
    CS_Low();
    /* Send program load command */
//    SendByte( FLASH_CMD_PP_LOAD );
	buff[0]=FLASH_CMD_PP_LOAD;
    /* Send flash address */
//    SendColAddr( col_address, addr_flag ); //发送 列地址
	buff[1]=((col_address>>8)|(addr_flag<<4));
	buff[2]=col_address;
	hdl_Spi_SendNbyte(buff,3);
	
    /* Send data to program */
    if(word_size>nd_page_wordsize)word_size=nd_page_wordsize;
    for(index=0;index<word_size;index++)
    {
        dat_temp  = *DataBuf;DataBuf+=1;
        buff[0] = ((uint32_t)dat_temp>>24)&0xff;
        buff[1] = ((uint32_t)dat_temp>>16)&0xff;
        buff[2] = ((uint32_t)dat_temp>>8)&0xff;
        buff[3] = dat_temp&0xff;
        hdl_Spi_SendNbyte(buff,4); 
    }
    // Chip select go high to end a flash command
    CS_High();
    return word_size;
}
uint16_t CMD_PP_LOAD8bit( uint16_t col_address,uint8_t addr_flag, uint8_t *DataBuf, uint16_t byte_size ) //8bit 页编程模式
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
/*
 * Program load Random Data Command
 */
/*
 * Function:       CMD_Program_Load_RandData
 * Arguments:      col_address, 16 bit col address
 *                 DataBuf, buffer of source data to program
 *                 byte_length, byte length of data to programm
 *                      addr_flag, define Plane select bit (only for 2Gb and 4Gb)
 * Description:    load program data without cache reset 
 * Return Message: Flash_Success               
 */
ReturnMsg CMD_PP_RAND_LOAD( uint16_t col_address, uint8_t *DataBuf, u32 Length, uint8_t addr_flag )
{
//    u32 i;
	uint8_t buff[4];
    // Chip select go low to start a flash command
    CS_Low();
    /* Send program load command */
//    SendByte( FLASH_CMD_PP_RAND_LOAD );
	buff[0]=FLASH_CMD_PP_RAND_LOAD;
    /* Send flash address */
//    SendColAddr( col_address, addr_flag );
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
 
 
/*
 * Function:       CMD_Program_Exec
 * Arguments:      address, 32 bit flash memory address
 * Description:    Enter block/page address,no data,execute
 * Return Message: Flash_Success,Flash_OperationTimeOut
 */
ReturnMsg CMD_PROGRAM_EXEC( u32 address )//程序执行命令 ,让写入数据生效
{
    uint8_t status;
	u8 sendbuf[4];
    // Chip select go low to start a flash command
    CS_Low();
    /* Send program execute command */
    SendByte( FLASH_CMD_PROGRAM_EXEC ); 
	sendbuf[0]=FLASH_CMD_PROGRAM_EXEC;
    /* Send flash address */
//    SendRowAddr( address>>12 );//发送行地址
	
	sendbuf[1]=(address>>28);
		sendbuf[2]=(address>>20);
		sendbuf[3]=(address>>12);
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
ReturnMsg CMD_BE( u32 flash_address ) //擦除flash
{
    uint8_t status,sendbuf[4];
    // Check flash address
    if( flash_address > FlashSize ) return Flash_AddrInvalid;
    /* Check flash is F_BUSY or not */
    if( CheckStatus( SR0_OIP ) != READY ) return Flash_F_BUSY;
    // Setting Write Enable Latch bit
    CMD_WREN();
    // Chip select go low to start a flash command
    CS_Low();
    //Write Block Erase command
    SendByte( FLASH_CMD_BE ); 
	sendbuf[0]=FLASH_CMD_BE;
//    SendRowAddr( flash_address>>12); //发送块地址信息
	
	sendbuf[1]=(flash_address>>28);
		sendbuf[2]=(flash_address>>20);
		sendbuf[3]=(flash_address>>12);
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
uint32_t   Get_nfaddr(uint32_t block){  return((uint32_t )block<<18);}
addr_more_t nand_flash_addr_convt(uint32_t addr) //物理地址转换
{
    addr_more_t tempaddr;
    addr_more_t flash_addr;
    tempaddr.all = addr;
    flash_addr.all =0;
    flash_addr.bit.Block_addr = tempaddr.bit.Block_addr;
    uint32_t temp = (2*tempaddr.bit.Page_addr);
    flash_addr.all+=(temp<<12);
    while(1)
    {
        if(tempaddr.bit.Col_addr>=nd_page_bytesize) //1 page
        {
            flash_addr.all+=0x1000;
            tempaddr.bit.Col_addr-=nd_page_bytesize;
        }
        else
        {
            break;
        }
    }
    flash_addr.bit.Col_addr = tempaddr.bit.Col_addr;
return flash_addr;
}
 
uint32_t flash_Cache_read_32bit(uint32_t addr,uint32_t *memory_addr,uint32_t word_len)
{
    addr_more_t flash_addr;
    ReturnMsg rets;
    uint16_t err=0;
    uint16_t size =0;
    uint32_t len=0;
    flash_addr = nand_flash_addr_convt(addr); 
    len =word_len;
    while(len)
    {
        rets =  CMD_RESET_OP();//rst nand flash sram
        if(Flash_Success!=rets){ dbg_printf("CMD_RESET_OP err,%d\r\n",rets);return 0;}
        rets =  CMD_READ( flash_addr.all ); 
        if(Flash_Success!=rets) dbg_printf("CMD_READ err,%d\r\n",rets);
        if(len>nd_page_wordsize){size =nd_page_wordsize; len-=nd_page_wordsize;}
        else {size = len;len=0;}
        err = CMD_READ_CACHE32bit( flash_addr.all&0xffff,0, memory_addr, size); 
        if(err!=size){dbg_printf("read len err ,is %d \r\n",err);return 0;}
        memory_addr+=size;//*2;
        flash_addr.all+=0x1000; 
    }
    return word_len;
}
// return 0: fail ,1:ok
int nflash_Verify_data32bit(uint32_t addr ,uint32_t *cmp ,uint32_t word_len)
{
    addr_more_t flash_addr;
    ReturnMsg rets;
    uint16_t size =0;
    uint32_t len=0;
    u32 index;
    uint32_t dat_temp =0;
    uint8_t buff[4];
     flash_addr = nand_flash_addr_convt(addr); 
     len =word_len;
     while(len)
     {
         rets =  CMD_RESET_OP();//rst nand flash sram
         if(Flash_Success!=rets){ dbg_printf("CMD_RESET_OP err,%d\r\n",rets);return 0;}
         rets =  CMD_READ( flash_addr.all ); 
         if(Flash_Success!=rets) dbg_printf("CMD_READ err,%d\r\n",rets);
         if(len>nd_page_wordsize){size =nd_page_wordsize; len-=nd_page_wordsize;}
         else {size = len;len=0;}
         {//read
          // Chip select go low to start a flash command
          CS_Low();
          // Write READ Cache command and address
          SendByte( FLASH_CMD_READ_CACHE ); 
          SendByte( 0); 
					buff[0] = FLASH_CMD_READ_CACHE;
					buff[1] = 0;
//          SendColAddr( flash_addr.all&0xffff, 0 ); 
					 
					buff[2]=(((flash_addr.all)>>8)|0);
					buff[3]=flash_addr.all;
					hdl_Spi_SendNbyte(buff,4);
					 
          // Set a loop to read data into buffer
          if(size>nd_page_wordsize)size =nd_page_wordsize;
          for( index=0; index < size; index++ )
          {
              // Read data one byte at a time
//              buff[0] = SendByte( 0 );
//              buff[1] = SendByte( 0 );
//              buff[2] = SendByte( 0 );
//              buff[3] = SendByte( 0 );
						hdl_Spi_RecvNbyte(buff,4);
              dat_temp = (uint32_t)buff[0]<<24;
              dat_temp |= (uint32_t)buff[1]<<16;
              dat_temp |= (uint32_t)buff[2]<<8;
              dat_temp |= (uint32_t)buff[3];
              if(dat_temp!=cmp[0])
              {
                  // Chip select go high to end a flash command
                  CS_High();
                  dbg_printf("src=%x \r\n",cmp[0]);
                  dbg_printf("flash=%x\r\n",dat_temp);
                  dbg_printf("write data fail,addr=%x\r\n",flash_addr.all);
                  return 0;
              }
              cmp++;
          }
          // Chip select go high to end a flash command
          CS_High();
         }
         flash_addr.all+=0x1000; //下一页
     }
    return 1;
}
//32 bit readwrite
uint32_t flash_progarm_write_32bit(uint32_t  block,uint32_t   *memory_addr,uint32_t  word_len )
{
    //1 block = (2K + 128 bytes) x 64 pages = 139,264 byte
     ReturnMsg rets;
     uint16_t err =0;
     uint8_t   st_reg1 = 0;
     addr_more_t flash_addr;
     uint32_t len;
     uint32_t *mem_dat;
     uint32_t size =0;
 //step 1
    rets =  CMD_RESET_OP();
    if(Flash_Success!=rets){ dbg_printf("CMD_RESET_OP err,%d\r\n",rets);return 0;}
     /* Clear the block protection bit:*/
     rets =CMD_GET_FEATURE( 0xa0, &st_reg1 );
     if (st_reg1 & 0x38)
     {
         rets =CMD_SET_FEATURE( 0xa0, (st_reg1&0x87) ); 
         if(Flash_Success!=rets){ dbg_printf("CMD_SET_FEATURE err,%d\r\n",rets);return 0;}
     }
 //step 2
     flash_addr.all =0;
     flash_addr.bit.Block_addr = block;
     rets =CMD_BE( flash_addr.all );
     if(Flash_Success!=rets) {dbg_printf("CMD_BE err,%d\r\n",rets);return 0;}
 //step 3
     len = (word_len>nd_block_wordsize)?nd_block_wordsize:word_len;
     mem_dat = memory_addr;
     while(len) //写入64个页面
     {
         /* Program data to flash memory */
         if(len>nd_page_wordsize) {size = nd_page_wordsize;len-=nd_page_wordsize;} //确定1page 写入长度
         else { size = len;len=0;}
//         dbg_printf("mem  : %lx \r\n",mem_dat[0]);//ok
         err = CMD_PP_LOAD32bit( flash_addr.all&0xffff,0, mem_dat, size);     // flash 页编程 ,一次最大写入 2176 byte
         if(err!=size){dbg_printf("load pp err ,err:%d\r\n",err);return 0;}
         mem_dat+=nd_page_wordsize;//*2;
         rets =  CMD_PROGRAM_EXEC( flash_addr.all ); //等待写入数据生效
         if(Flash_Success!=rets) dbg_printf("CMD_PROGRAM_EXEC err,%d\r\n",rets);
         flash_addr.bit.Page_addr++;//页地址递增
     }
     CMD_WRDI(); //关闭擦写使能
// Verify  data ...
     len = (word_len>nd_block_wordsize)?nd_block_wordsize:word_len;
     flash_addr.all =0;
     flash_addr.bit.Block_addr = block; //初始化块 地址
     err = nflash_Verify_data32bit(flash_addr.all,memory_addr,len);
     if(err<1) return 0;
     else return len;//ok
}
 
 
// 随机读取flash
uint32_t flash_Cache_read_8bit(uint32_t addr,uint8_t *memory_addr,uint32_t byte_len)
{
    addr_more_t flash_addr;
    ReturnMsg rets;
    uint16_t err=0;
    uint16_t size =0;
    uint32_t len=0;
    flash_addr = nand_flash_addr_convt(addr); 
    len =byte_len;
    while(len)
    {
        rets =  CMD_RESET_OP();//rst nand flash sram
        if(Flash_Success!=rets){ dbg_printf("CMD_RESET_OP err,%d\r\n",rets);return 0;}
        rets =  CMD_READ( flash_addr.all ); 
        if(Flash_Success!=rets) dbg_printf("CMD_READ err,%d\r\n",rets);
        if(len>nd_page_bytesize){size =nd_page_bytesize; len-=nd_page_bytesize;}
        else {size = len;len=0;}
        err = CMD_READ_CACHE8bit( flash_addr.all&0xffff,0, memory_addr, size); 
        if(err!=size){dbg_printf("read len err ,is %d \r\n",err);return 0;}
        memory_addr+=size;
        flash_addr.all+=0x1000; //下一页
    }
    return byte_len;
 
}
 
 
// return 0: fail ,1:ok
int nflash_Verify_data8bit(uint32_t addr ,uint8_t *cmp ,uint32_t byte_len)
{
    addr_more_t flash_addr;
    ReturnMsg rets;
    uint16_t size =0;
    uint32_t len=0;
    u32 index;
    uint8_t dat_temp =0;
    uint8_t buff8,buff[4];
     flash_addr = nand_flash_addr_convt(addr); 
     len =byte_len;
     while(len)
     {
         rets =  CMD_RESET_OP();//rst nand flash sram
         if(Flash_Success!=rets){ dbg_printf("CMD_RESET_OP err,%d\r\n",rets);return 0;}
         rets =  CMD_READ( flash_addr.all ); 
         if(Flash_Success!=rets) dbg_printf("CMD_READ err,%d\r\n",rets);
         if(len>nd_page_bytesize){size =nd_page_bytesize; len-=nd_page_bytesize;}
         else {size = len;len=0;}
         {//read
          // Chip select go low to start a flash command
          CS_Low();
          // Write READ Cache command and address
//          SendByte( FLASH_CMD_READ_CACHE ); 
//          SendByte( 0); 
//          SendColAddr( flash_addr.all&0xffff, 0 ); 
					buff[0]= FLASH_CMD_READ_CACHE;
					buff[1]=0;
					buff[2]=(((flash_addr.all)>>8)|0);
					buff[3]=flash_addr.all;
					hdl_Spi_SendNbyte(buff,4);
					 
          // Set a loop to read data into buffer
          if(size>nd_page_bytesize)size =nd_page_bytesize;
          for( index=0; index < size; index++ )
          {
              // Read data one byte at a time
 //             buff8 = SendByte( 0 )&0xff;
 			hdl_Spi_RecvNbyte(&buff8,4);
						
              dat_temp = cmp[0]&0xff;cmp++;
              if(dat_temp!=buff8)
              {
                  // Chip select go high to end a flash command
                  CS_High();
                  dbg_printf("src=%x \r\n",cmp[0]);
                  dbg_printf("flash=%x\r\n",buff8);
                  dbg_printf("write data fail,addr=%x\r\n",flash_addr.all);
                  return 0;
              }
 
          }
          // Chip select go high to end a flash command
          CS_High();
         }
         flash_addr.all+=0x1000; //下一页
     }
    return 1;
}
//8 bit readwrite
// 块编程 ,擦除地址按块擦除
uint32_t flash_progarm_write_8bit(uint32_t  block,uint8_t   *memory_addr,uint32_t  byte_len )
{
    //1 block = (2K + 128 bytes) x 64 pages = 139,264 byte
    ReturnMsg rets;
    uint8_t * pta =memory_addr;
    uint16_t err =0;
    uint8_t   st_reg1 = 0;
    addr_more_t flash_addr;
    uint32_t len;
    uint32_t size =0;
//step 1
    /* Clear the block protection bit:清除块保护位*/
    rets =CMD_GET_FEATURE( 0xa0, &st_reg1 );
    if (st_reg1 & 0x38)
    {
        rets =CMD_SET_FEATURE( 0xa0, (st_reg1&0x87) );
        if(Flash_Success!=rets){ dbg_printf("CMD_SET_FEATURE err,%d\r\n",rets);return 0;}
    }
//step 2
    flash_addr.all =0;
    flash_addr.bit.Block_addr = block;//擦除这个块
    rets =CMD_BE( flash_addr.all );
    if(Flash_Success!=rets) {dbg_printf("CMD_BE err,%d\r\n",rets);return 0;}
//step 3
    len = (byte_len>nd_block_bytesize)?nd_block_bytesize:byte_len;
    pta = memory_addr;
    while(len) //写入64个页面
    {
        /* Program data to flash memory */
        if(len>nd_page_bytesize) {size = nd_page_bytesize;len-=nd_page_bytesize;} //确定1page 写入长度
        else { size = len;len=0;}
        err = CMD_PP_LOAD8bit( flash_addr.all,0, pta, size);     // flash 页编程 ,一次最大写入 2176 byte
        if(err!=size){dbg_printf("load pp err ,err:%d\r\n",err);return 0;}
        pta+=size;
        rets =  CMD_PROGRAM_EXEC( flash_addr.all ); //等待写入数据生效
        if(Flash_Success!=rets) dbg_printf("CMD_PROGRAM_EXEC err,%d\r\n",rets);
        flash_addr.bit.Page_addr++;//页地址递增
    }
    CMD_WRDI(); //关闭擦写使能
// Verify  data ...
     len = (byte_len>nd_block_bytesize)?nd_block_bytesize:byte_len;
     flash_addr.all =0;
     flash_addr.bit.Block_addr = block; //初始化块 地址
     err = nflash_Verify_data8bit(flash_addr.all,memory_addr,len);
     if(err<1) return 0;
     else return len;//ok
}
//————————————————
//版权声明：本文为CSDN博主「kensey」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/qq_40545297/article/details/102676925

