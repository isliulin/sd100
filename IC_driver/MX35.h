

#ifndef __MX35_H
#define __MX35_H

#include "mydefine.h"

#define MX35_Configuration	0xB0

#define MX35_Status	0xC0

//SR[0] (OIP) Operation in progress		1 忙碌 0空闲
//The bit value indicates whether the device is busy in operations of read/ program execute/ erase/ reset command.  1: Busy,  0: Ready   

//SR[1] (WEL) Write enable latch  1 写使能  0 写禁能
//The bit value indicates whether the device is set to internal write enable latch. When WEL bit sets to 1, which means the internal write enable latch is set, and then the device can accept program/ erase/write status register instruction.  1: write enable,  0: not write enable    The bit value will be cleared (as "0") by issuing Write Disable command(04h).

//SR[2]  (ERS_Fail) Erase fail  1 擦除失败  0 擦除成功
//The bit value shows the status of erase failure or if host erase any invalid address or protected area (including protected blocks or protected Secure OTP area). 0: Passed, 1: Failed The bit value will be cleared (as "0") by RESET command or at the beginning of the block erase command operation.

//SR[3]  (PGM_Fail) Program fail   1 写入失败 0 写入成功
//The bit value shows the status of program failure or if host program any invalid address or protected area (including protected blocks or protected Secure OTP area). 0: Passed, 1: Failed The bit value will be cleared (as "0") by RESET command or during the program execute command operation.

//SR[6] (CRBSY) Cache  Status Bit  1 忙碌 0空闲
//The bit value indicates whether the internal cache is busy in Page Read Cache Sequential command.        1: Busy- internal cache is busy on data transfer        0: Ready- device is ready for cache data out




#define Block_Protection 0xA0

//Protection A0h   BPRWD1 Reserved BP2 BP1 BP0 Invert Complementary SP
//BP2 BP1 BP0 Invert Complementary 5位确定受保护的区块
//BPRWD1	//如果BPRWD已启用且WP#引脚为低，则无法更改块保护寄存器
//SP位用于固态保护。一旦SP位设置为1，其余的保护位（BPx位、反转位、互补位）在当前电源循环期间不能改变。


//参考 https://blog.csdn.net/qq_40545297/article/details/102676925?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522158391183119724846448656%2522%252C%2522scm%2522%253A%252220140713.130056874..%2522%257D&request_id=158391183119724846448656&biz_id=0&utm_source=distribute.pc_search_result.none-task

//#define MX35LF2G14AC            1
#define MX35LF1GE14AB            1 
 
 
/*** MX35 series command hex code definition ***/
//ID comands
#define    FLASH_CMD_RDID           			0x9F    //RDID (Read Identification)
#define    FLASH_CMD_GET_FEATURE    			0x0F    //Get features
#define    FLASH_CMD_SET_FEATURE    			0x1F    //Set features
#define    FLASH_CMD_READ           			0x13    //Array Read
#define    FLASH_CMD_READ_CACHE     			0x03    //Read From Cache
#define    FLASH_CMD_READ_CACHE2    			0x3B    //Read From Cache*2
#define    FLASH_CMD_READ_CACHE4    			0x6B    //Read From Cache*4
#define    FLASH_CMD_READ_CACHE_SEQUENTIAL   0x31    //Read From Cache Sequential
#define    FLASH_CMD_READ_CACHE_END    		0x3F    //Read From Cache End
#define    FLASH_CMD_WREN           			0x06    //Write Enable
#define    FLASH_CMD_WRDI           			0x04    //Write Disable
#define    FLASH_CMD_PP_LOAD        			0x02    //Page Program Load
#define    FLASH_CMD_PP_RAND_LOAD   			0x84    //Page Program Random Input
#define    FLASH_CMD_4PP_LOAD       			0x32    //Quad IO Page Program Load
#define    FLASH_CMD_4PP_RAND_LOAD  			0x34    //Quad IO Page Program Random Input
#define    FLASH_CMD_PROGRAM_EXEC   			0x10    //Program Execute
#define    FLASH_CMD_BE             			0xD8    //BLock Erase
 
#define     BYTE_MASK                           0xFF
// system flags
#define  PASS        0
#define  F_FAIL        1
#define  F_BUSY        0
#define  READY       1
#define  PROTECTED   0
#define  UNPROTECTED 1
 
#define  TIMEOUT    0
#define  TIMENOTOUT 1
 
// Flash control register mask define
// status register
// status register [7:0]
#define  SR0_OIP               0x01	//
#define  SR1_WEL               0x02
#define  SR2_EraseF_FAIL         0x04
#define  SR2_ProgramF_FAIL       0x08
 
 
#ifdef MX35LF2G14AC
#define    FlashID          0xB248 //0xc220
#define    FlashSize        0x10000000      // 256 MiB
#endif

#ifdef MX35LF1GE14AB
#define    FlashID          0xB248 //0xc220
#define    Flash_Page_Size        0xffff      // 256 MiB  0xffff  1G-页16位, 2G-页17位(页6位,block 11位), 字12位 1024+64, 字转为 3位
#endif

#define         nd_page_wordsize        (512)//2048+64 = 2112byte/4  = 528word
#define         nd_page_bytesize        (2048)// = 2112byte
// 1块数据大小
#define         nd_block_wordsize       (32768)//((2048)*64) = 131,072byte /4 = 32768word
#define         nd_block_bytesize       (131072)
 
u32   Get_nfaddr(u32 block);
#define  GET_FLASH_ADDR      Get_nfaddr
 
 
struct nandflash_addrst
{
    u32 Col_addr :12;//  bit11:bit0 => 数据地址(Col列地址)
    u32 Page_addr :6;//  bit17:bit12=> 页地址信息(最大63)
    u32 Block_addr :11;//  bit31:bit18=> 块地址信息(bit28:bit18=>11bit有效)
    u32 Rsvd_addr :3; //保留
};
 
typedef union AddrMore_st
{
    u32 all;
    struct nandflash_addrst bit;
} addr_more_t;
 
/* Return Message */
typedef enum{
    Flash_Success,
    Flash_F_BUSY,
    Flash_OperationTimeOut,
    Flash_ProgramF_FAILed,
    Flash_EraseF_FAILed,
    Flash_ReadIDF_FAILed,
    Flash_CmdInvalid,
    Flash_DataInvalid,
    Flash_AddrInvalid,
    Flash_QuadNotEnable
}ReturnMsg;
 
 
// Flash status structure define
struct sFlashStatus{
    /* Mode Register:
     * Bit  Description
     * -------------------------
     *  7   RYBY enable
     *  6   Reserved
     *  5   Reserved
     *  4   Reserved
     *  3   Reserved
     *  2   Reserved
     *  1   Parallel mode enable
     *  0   QPI mode enable
    */
    u8    ModeReg;
    int     ArrangeOpt;
};


#endif /*__MX35_H*/
//————————————————
//版权声明：本文为CSDN博主「kensey」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/qq_40545297/article/details/102676925

