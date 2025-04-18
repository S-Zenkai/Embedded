#include "bsp_mfrc522.h"
#include "bsp_SisTick.h"

static  uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

//GPIO初始化
static void SPI_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//开时钟
	MFRC522_SPI_GPIO_APBxClock_FUN(MFRC522_SPI_GPIO_CLK, ENABLE);
	
	//GPIO_SCK配置
	GPIO_InitStructure.GPIO_Pin=MFRC522_SPI_GPIO_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MFRC522_SPI_GPIO_SCK_PORT, &GPIO_InitStructure);
	
	//GPIO_MOSI配置
	GPIO_InitStructure.GPIO_Pin=MFRC522_SPI_GPIO_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MFRC522_SPI_GPIO_MOSI_PORT, &GPIO_InitStructure);
	
	//GPIO_MISO配置
	GPIO_InitStructure.GPIO_Pin=MFRC522_SPI_GPIO_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MFRC522_SPI_GPIO_MISO_PORT, &GPIO_InitStructure);
	
	//GPIO_CS配置
	GPIO_InitStructure.GPIO_Pin=MFRC522_SPI_GPIO_CS_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MFRC522_SPI_GPIO_CS_PORT, &GPIO_InitStructure);
	
	//GPIO_RST配置
	GPIO_InitStructure.GPIO_Pin=MFRC522_SPI_GPIO_RST_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MFRC522_SPI_GPIO_RST_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
}


//SPI初始化
static void SPI_Config(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	
	//开时钟
	MFRC522_SPI_APBxClock_FUN(MFRC522_SPI_CLK, ENABLE);
	
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;//配置通信频率要少于10Mhz
	//选择mode0
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
	SPI_InitStructure.SPI_CRCPolynomial=0;
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(MFRC522_SPIx, &SPI_InitStructure);
	SPI_Cmd(MFRC522_SPIx,ENABLE);
}


void SPI_MFRC522_Init(void)
{
	SPI_GPIO_Config();
	SPI_Config();
//	RC522_Reset_Disable();	
//	RC522_CS_Disable();
}



#if 1
uint8_t SPI_Flash_WritByte(uint8_t Data)
{
	uint16_t WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(0);
			break;
		}
	}
	SPI_I2S_SendData(MFRC522_SPIx, Data);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(1);
			break;
		}
	}
	return SPI_I2S_ReceiveData(MFRC522_SPIx);
}

void WriteRawRC(uint8_t Addr,uint8_t Data)
{
	GPIO_ResetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	uint8_t WriAddr=0;
	WriAddr=(Addr<<1)&0x7E;
	Delay_SisTick_us(10);
	SPI_Flash_WritByte(WriAddr);
	Delay_SisTick_us(10);
	SPI_Flash_WritByte(Data);
	GPIO_SetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
}

uint8_t ReadRawRC(uint8_t Addr)
{
	uint8_t ret=0;
	GPIO_ResetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	uint8_t WriAddr=0;
	WriAddr=((Addr<<1)&0x7E)|0x80;
	Delay_SisTick_us(10);
	SPI_Flash_WritByte(WriAddr);
	Delay_SisTick_us(10);
	ret=SPI_Flash_WritByte(0);
	GPIO_SetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	return ret;
}

#elif 0
//向MFRC522写入一个字节
void WriteRawRC(uint8_t Addr,uint8_t Data)
{
	GPIO_ResetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	uint8_t WriAddr=0;
	WriAddr=(Addr<<1)&0x7E;
	uint16_t WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(0);
			break;
		}
	}
	SPI_I2S_SendData(MFRC522_SPIx, WriAddr);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(1);
			break;
		}
	}
	SPI_I2S_ReceiveData(MFRC522_SPIx);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(0);
			break;
		}
	}
	SPI_I2S_SendData(MFRC522_SPIx, Data);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
		WaitTime--;
		if(WaitTime==0)
		{
			SPI_TIMEOUT_UserCallback(1);
			break;
		}
	}
	SPI_I2S_ReceiveData(MFRC522_SPIx);
	GPIO_SetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
}


//从MFRC522读一个字节
uint8_t ReadRawRC(uint8_t Addr)
{
	uint16_t ret=0;
	GPIO_ResetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	uint8_t WriAddr=0;
	WriAddr=((Addr<<1)&0x7E)|0x80;
	uint16_t WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
//		WaitTime--;
//		if(WaitTime==0)
//		{
//			SPI_TIMEOUT_UserCallback(0);
//			break;
//		}
	}
	SPI_I2S_SendData(MFRC522_SPIx, WriAddr);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
//		WaitTime--;
//    if(WaitTime==0)
//		{
//			SPI_TIMEOUT_UserCallback(0);
//			break;
//		}
	}
	SPI_I2S_ReceiveData(MFRC522_SPIx);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_TXE)==RESET)
	{
//		WaitTime--;
//		if(WaitTime==0)
//		{
//			SPI_TIMEOUT_UserCallback(0);
//			break;
//		}
	}
	SPI_I2S_SendData(MFRC522_SPIx, 0);
	WaitTime=SPI_Flag_WaitTime;
	while(SPI_I2S_GetFlagStatus(MFRC522_SPIx, SPI_I2S_FLAG_RXNE)==RESET)
	{
//		WaitTime--;
//		if(WaitTime==0)
//		{
//			SPI_TIMEOUT_UserCallback(1);
//			break;
//		}
	}
	ret=SPI_I2S_ReceiveData(MFRC522_SPIx);
	GPIO_SetBits(MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN);
	return ret;
}

#elif 0

/**
  * @brief  向RC522发送1 Byte 数据
  * @param  byte，要发送的数据
  * @retval RC522返回的数据
  */
//#define          RC522_CS_Enable()         GPIO_ResetBits ( MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN )
//#define          RC522_CS_Disable()        GPIO_SetBits ( MFRC522_SPI_GPIO_CS_PORT, MFRC522_SPI_GPIO_CS_PIN )

#define          RC522_Reset_Enable()      GPIO_ResetBits( MFRC522_SPI_GPIO_RST_PORT, MFRC522_SPI_GPIO_RST_PIN )
#define          RC522_Reset_Disable()     GPIO_SetBits ( MFRC522_SPI_GPIO_RST_PORT, MFRC522_SPI_GPIO_RST_PIN )

#define          RC522_SCK_0()             GPIO_ResetBits( MFRC522_SPI_GPIO_SCK_PORT, MFRC522_SPI_GPIO_SCK_PIN )
#define          RC522_SCK_1()             GPIO_SetBits ( MFRC522_SPI_GPIO_SCK_PORT, MFRC522_SPI_GPIO_SCK_PIN )
#define          RC522_MOSI_0()            GPIO_ResetBits( MFRC522_SPI_GPIO_MOSI_PORT, MFRC522_SPI_GPIO_MOSI_PIN )
#define          RC522_MOSI_1()            GPIO_SetBits ( MFRC522_SPI_GPIO_MOSI_PORT, MFRC522_SPI_GPIO_MOSI_PIN )
#define          RC522_MISO_GET()          GPIO_ReadInputDataBit ( MFRC522_SPI_GPIO_MISO_PORT, MFRC522_SPI_GPIO_MISO_PIN )
#define   RC522_DELAY()  Delay_SisTick_us ( 2 )
void SPI_RC522_SendByte ( uint8_t byte )
{
  uint8_t counter;

  for(counter=0;counter<8;counter++)
  {     
    if ( byte & 0x80 )
      RC522_MOSI_1 ();
    else 
      RC522_MOSI_0 ();
    
    RC522_DELAY();
    RC522_SCK_0 ();
    
    RC522_DELAY();
    RC522_SCK_1();
    
    RC522_DELAY();
    byte <<= 1; 
  } 	
}


/**
  * @brief  从RC522发送1 Byte 数据
  * @param  无
  * @retval RC522返回的数据
  */
uint8_t SPI_RC522_ReadByte ( void )
{
  uint8_t counter;
  uint8_t SPI_Data;

  for(counter=0;counter<8;counter++)
  {
    SPI_Data <<= 1;
    RC522_SCK_0 ();
   
    RC522_DELAY();
    if ( RC522_MISO_GET() == 1)
     SPI_Data |= 0x01;
    
    RC522_DELAY();
    RC522_SCK_1 ();
    
    RC522_DELAY();
  }
  return SPI_Data;
	
}



/**
  * @brief  读RC522寄存器
  * @param  ucAddress，寄存器地址
  * @retval 寄存器的当前值
  */
uint8_t ReadRawRC ( uint8_t ucAddress )
{
	uint8_t ucAddr, ucReturn;
	
	ucAddr = ( ( ucAddress << 1 ) & 0x7E ) | 0x80;	
	RC522_CS_Enable();
  
	SPI_RC522_SendByte ( ucAddr );
	ucReturn = SPI_RC522_ReadByte ();
  
	RC522_CS_Disable();
	
	return ucReturn;	
}

/**
  * @brief  写RC522寄存器
  * @param  ucAddress，寄存器地址
  * @param  ucValue，写入寄存器的值
  * @retval 无
  */
void WriteRawRC ( uint8_t ucAddress, uint8_t ucValue )
{  
	uint8_t ucAddr;
	
	ucAddr = ( ucAddress << 1 ) & 0x7E;	
	RC522_CS_Enable();
	
	SPI_RC522_SendByte ( ucAddr );	
	SPI_RC522_SendByte ( ucValue );
  
	RC522_CS_Disable();		
}
#endif

/**
  * @brief  对RC522寄存器置位
  * @param  ucReg，寄存器地址
  * @param   ucMask，置位值
  * @retval 无
  */
void SetBitMask ( uint8_t ucReg, uint8_t ucMask )  
{
  uint8_t ucTemp;

  ucTemp = ReadRawRC ( ucReg );
  WriteRawRC ( ucReg, ucTemp | ucMask ); // set bit mask
}


/**
  * @brief  对RC522寄存器清位
  * @param  ucReg，寄存器地址
  * @param  ucMask，清位值
  * @retval 无
  */
void ClearBitMask ( uint8_t ucReg, uint8_t ucMask )  
{
  uint8_t ucTemp;

  ucTemp = ReadRawRC ( ucReg );
  WriteRawRC ( ucReg, ucTemp & ( ~ ucMask) ); // clear bit mask
}


/**
  * @brief  开启天线 
  * @param  无
  * @retval 无
  */
void PcdAntennaOn ( void )
{
  uint8_t uc;

  uc = ReadRawRC ( TxControlReg );
  if ( ! ( uc & 0x03 ) )
   SetBitMask(TxControlReg, 0x03);		
}


/**
  * @brief  关闭天线
  * @param  无
  * @retval 无
  */
void PcdAntennaOff ( void )
{
  ClearBitMask ( TxControlReg, 0x03 );	
}


/**
  * @brief  复位RC522 
  * @param  无
  * @retval 无
  */
void PcdReset ( void )
{
	RC522_Reset_Disable();
	Delay_SisTick_us ( 1 );
	
	RC522_Reset_Enable();
	Delay_SisTick_us ( 1 );
	
	RC522_Reset_Disable();
	Delay_SisTick_us ( 1 );
	
	WriteRawRC ( CommandReg, 0x0f );
	
	while ( ReadRawRC ( CommandReg ) & 0x10 );
	
	Delay_SisTick_us ( 1 );
  
	//定义发送和接收常用模式 和Mifare卡通讯，CRC初始值0x6363
  WriteRawRC ( ModeReg, 0x3D );        
	
  WriteRawRC ( TReloadRegL, 30 );      //16位定时器低位    
	WriteRawRC ( TReloadRegH, 0 );			 //16位定时器高位
	
  WriteRawRC ( TModeReg, 0x8D );			 //定义内部定时器的设置
	
  WriteRawRC ( TPrescalerReg, 0x3E );	 //设置定时器分频系数
	
	WriteRawRC ( TxAutoReg, 0x40 );			 //调制发送信号为100%ASK	
	
}



/**
  * @brief  设置RC522的工作方式
  * @param  ucType，工作方式
  * @retval 无
  */
void M500PcdConfigISOType ( uint8_t ucType )
{
	if ( ucType == 'A')                     //ISO14443_A
  {
		ClearBitMask ( Status2Reg, 0x08 );
		
    WriteRawRC ( ModeReg, 0x3D );         //3F
		
		WriteRawRC ( RxSelReg, 0x86 );        //84
		
		WriteRawRC( RFCfgReg, 0x7F );         //4F
		
		WriteRawRC( TReloadRegL, 30 );        
		
		WriteRawRC ( TReloadRegH, 0 );
		
		WriteRawRC ( TModeReg, 0x8D );
		
		WriteRawRC ( TPrescalerReg, 0x3E );
		
		Delay_SisTick_us ( 2 );
		
		PcdAntennaOn ();//开天线
		
   }	 
}



/**
  * @brief  通过RC522和ISO14443卡通讯
  * @param  ucCommand，RC522命令字
  * @param  pInData，通过RC522发送到卡片的数据
  * @param  ucInLenByte，发送数据的字节长度
  * @param  pOutData，接收到的卡片返回数据
  * @param  pOutLenBit，返回数据的位长度
  * @retval 状态值= MI_OK，成功
  */
char PcdComMF522 ( uint8_t ucCommand,
                   uint8_t * pInData, 
                   uint8_t ucInLenByte, 
                   uint8_t * pOutData,
                   uint32_t * pOutLenBit )		
{
  char cStatus = MI_ERR;
  uint8_t ucIrqEn   = 0x00;
  uint8_t ucWaitFor = 0x00;
  uint8_t ucLastBits;
  uint8_t ucN;
  uint32_t ul;

  switch ( ucCommand )
  {
     case PCD_AUTHENT:		  //Mifare认证
        ucIrqEn   = 0x12;		//允许错误中断请求ErrIEn  允许空闲中断IdleIEn
        ucWaitFor = 0x10;		//认证寻卡等待时候 查询空闲中断标志位
        break;
     
     case PCD_TRANSCEIVE:		//接收发送 发送接收
        ucIrqEn   = 0x77;		//允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
        ucWaitFor = 0x30;		//寻卡等待时候 查询接收中断标志位与 空闲中断标志位
        break;
     
     default:
       break;     
  }
  //IRqInv置位管脚IRQ与Status1Reg的IRq位的值相反 
  WriteRawRC ( ComIEnReg, ucIrqEn | 0x80 );
  //Set1该位清零时，CommIRqReg的屏蔽位清零
  ClearBitMask ( ComIrqReg, 0x80 );	 
  //写空闲命令
  WriteRawRC ( CommandReg, PCD_IDLE );		 
  
  //置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除
  SetBitMask ( FIFOLevelReg, 0x80 );			

  for ( ul = 0; ul < ucInLenByte; ul ++ )
    WriteRawRC ( FIFODataReg, pInData [ ul ] ); //写数据进FIFOdata
    
  WriteRawRC ( CommandReg, ucCommand );					//写命令


  if ( ucCommand == PCD_TRANSCEIVE )
    
    //StartSend置位启动数据发送 该位与收发命令使用时才有效
    SetBitMask(BitFramingReg,0x80);  				  

  ul = 1000;                             //根据时钟频率调整，操作M1卡最大等待时间25ms

  do 														         //认证 与寻卡等待时间	
  {
       ucN = ReadRawRC ( ComIrqReg );		 //查询事件中断
       ul --;
  } while ( ( ul != 0 ) && ( ! ( ucN & 0x01 ) ) && ( ! ( ucN & ucWaitFor ) ) );	

  ClearBitMask ( BitFramingReg, 0x80 );	 //清理允许StartSend位

  if ( ul != 0 )
  {
    //读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
    if ( ! ( ReadRawRC ( ErrorReg ) & 0x1B ) )	
    {
      cStatus = MI_OK;
      
      if ( ucN & ucIrqEn & 0x01 )				//是否发生定时器中断
        cStatus = MI_NOTAGERR;   
        
      if ( ucCommand == PCD_TRANSCEIVE )
      {
        //读FIFO中保存的字节数
        ucN = ReadRawRC ( FIFOLevelReg );		          
        
        //最后接收到得字节的有效位数
        ucLastBits = ReadRawRC ( ControlReg ) & 0x07;	
        
        if ( ucLastBits )
          
          //N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
          * pOutLenBit = ( ucN - 1 ) * 8 + ucLastBits;   	
        else
          * pOutLenBit = ucN * 8;      //最后接收到的字节整个字节有效
        
        if ( ucN == 0 )		
          ucN = 1;    
        
        if ( ucN > MAXRLEN )
          ucN = MAXRLEN;   
        
        for ( ul = 0; ul < ucN; ul ++ )
          pOutData [ ul ] = ReadRawRC ( FIFODataReg );   
        
        }        
    }   
    else
      cStatus = MI_ERR;       
  }

  SetBitMask ( ControlReg, 0x80 );           // stop timer now
  WriteRawRC ( CommandReg, PCD_IDLE ); 
   
  return cStatus;
}

/**
  * @brief 寻卡
  * @param  ucReq_code，寻卡方式 = 0x52，寻感应区内所有符合14443A标准的卡；
            寻卡方式= 0x26，寻未进入休眠状态的卡
  * @param  pTagType，卡片类型代码
             = 0x4400，Mifare_UltraLight
             = 0x0400，Mifare_One(S50)
             = 0x0200，Mifare_One(S70)
             = 0x0800，Mifare_Pro(X))
             = 0x4403，Mifare_DESFire
  * @retval 状态值= MI_OK，成功
  */
char PcdRequest ( uint8_t ucReq_code, uint8_t * pTagType )
{
  char cStatus;  
  uint8_t ucComMF522Buf [ MAXRLEN ]; 
  uint32_t ulLen;

  //清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
  ClearBitMask ( Status2Reg, 0x08 );
	//发送的最后一个字节的 七位
  WriteRawRC ( BitFramingReg, 0x07 );
  //TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号
  SetBitMask ( TxControlReg, 0x03 );	

  ucComMF522Buf [ 0 ] = ucReq_code;		//存入 卡片命令字

  cStatus = PcdComMF522 ( PCD_TRANSCEIVE,	
                          ucComMF522Buf,
                          1, 
                          ucComMF522Buf,
                          & ulLen );	//寻卡  

  if ( ( cStatus == MI_OK ) && ( ulLen == 0x10 ) )	//寻卡成功返回卡类型 
  {    
     * pTagType = ucComMF522Buf [ 0 ];
     * ( pTagType + 1 ) = ucComMF522Buf [ 1 ];
  }

  else
   cStatus = MI_ERR;

  return cStatus;	 
}

/**
  * @brief  防冲撞
  * @param  pSnr，卡片序列号，4字节
  * @retval 状态值= MI_OK，成功
  */
char PcdAnticoll ( uint8_t * pSnr )
{
  char cStatus;
  uint8_t uc, ucSnr_check = 0;
  uint8_t ucComMF522Buf [ MAXRLEN ]; 
  uint32_t ulLen;
  
  //清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
  ClearBitMask ( Status2Reg, 0x08 );
  //清理寄存器 停止收发
  WriteRawRC ( BitFramingReg, 0x00);	
	//清ValuesAfterColl所有接收的位在冲突后被清除
  ClearBitMask ( CollReg, 0x80 );			  
 
  ucComMF522Buf [ 0 ] = 0x93;	          //卡片防冲突命令
  ucComMF522Buf [ 1 ] = 0x20;
 
  cStatus = PcdComMF522 ( PCD_TRANSCEIVE, 
                          ucComMF522Buf,
                          2, 
                          ucComMF522Buf,
                          & ulLen);      //与卡片通信

  if ( cStatus == MI_OK)		            //通信成功
  {
    for ( uc = 0; uc < 4; uc ++ )
    {
       * ( pSnr + uc )  = ucComMF522Buf [ uc ]; //读出UID
       ucSnr_check ^= ucComMF522Buf [ uc ];
    }
    
    if ( ucSnr_check != ucComMF522Buf [ uc ] )
      cStatus = MI_ERR;    				 
  }
  
  SetBitMask ( CollReg, 0x80 );
      
  return cStatus;		
}


/**
  * @brief  用RC522计算CRC16
  * @param  pIndata，计算CRC16的数组
  * @param  ucLen，计算CRC16的数组字节长度
  * @param  pOutData，存放计算结果存放的首地址
  * @retval 无
  */
void CalulateCRC ( uint8_t * pIndata, 
                 uint8_t ucLen, 
                 uint8_t * pOutData )
{
  uint8_t uc, ucN;

  ClearBitMask(DivIrqReg,0x04);

  WriteRawRC(CommandReg,PCD_IDLE);

  SetBitMask(FIFOLevelReg,0x80);

  for ( uc = 0; uc < ucLen; uc ++)
    WriteRawRC ( FIFODataReg, * ( pIndata + uc ) );   

  WriteRawRC ( CommandReg, PCD_CALCCRC );

  uc = 0xFF;

  do 
  {
      ucN = ReadRawRC ( DivIrqReg );
      uc --;
  } while ( ( uc != 0 ) && ! ( ucN & 0x04 ) );
  
  pOutData [ 0 ] = ReadRawRC ( CRCResultRegL );
  pOutData [ 1 ] = ReadRawRC ( CRCResultRegM );		
}


/**
  * @brief  选定卡片
  * @param  pSnr，卡片序列号，4字节
  * @retval 状态值= MI_OK，成功
  */
char PcdSelect ( uint8_t * pSnr )
{
  char ucN;
  uint8_t uc;
  uint8_t ucComMF522Buf [ MAXRLEN ]; 
  uint32_t  ulLen;
  
  
  ucComMF522Buf [ 0 ] = PICC_ANTICOLL1;
  ucComMF522Buf [ 1 ] = 0x70;
  ucComMF522Buf [ 6 ] = 0;

  for ( uc = 0; uc < 4; uc ++ )
  {
    ucComMF522Buf [ uc + 2 ] = * ( pSnr + uc );
    ucComMF522Buf [ 6 ] ^= * ( pSnr + uc );
  }
  
  CalulateCRC ( ucComMF522Buf, 7, & ucComMF522Buf [ 7 ] );

  ClearBitMask ( Status2Reg, 0x08 );

  ucN = PcdComMF522 ( PCD_TRANSCEIVE,
                     ucComMF522Buf,
                     9,
                     ucComMF522Buf, 
                     & ulLen );
  
  if ( ( ucN == MI_OK ) && ( ulLen == 0x18 ) )
    ucN = MI_OK;  
  else
    ucN = MI_ERR;    
  
  return ucN;		
}




/**
  * @brief  验证卡片密码
  * @param  ucAuth_mode，密码验证模式= 0x60，验证A密钥，
            密码验证模式= 0x61，验证B密钥
  * @param  uint8_t ucAddr，块地址
  * @param  pKey，密码 
  * @param  pSnr，卡片序列号，4字节
  * @retval 状态值= MI_OK，成功
  */
char PcdAuthState ( uint8_t ucAuth_mode, 
                    uint8_t ucAddr, 
                    uint8_t * pKey,
                    uint8_t * pSnr )
{
  char cStatus;
  uint8_t uc, ucComMF522Buf [ MAXRLEN ];
  uint32_t ulLen;
  

  ucComMF522Buf [ 0 ] = ucAuth_mode;
  ucComMF522Buf [ 1 ] = ucAddr;

  for ( uc = 0; uc < 6; uc ++ )
    ucComMF522Buf [ uc + 2 ] = * ( pKey + uc );   

  for ( uc = 0; uc < 6; uc ++ )
    ucComMF522Buf [ uc + 8 ] = * ( pSnr + uc );   

  cStatus = PcdComMF522 ( PCD_AUTHENT,
                          ucComMF522Buf, 
                          12,
                          ucComMF522Buf,
                          & ulLen );

  if ( ( cStatus != MI_OK ) || ( ! ( ReadRawRC ( Status2Reg ) & 0x08 ) ) )
    cStatus = MI_ERR;   
    
  return cStatus;
}


/**
  * @brief  写数据到M1卡
  * @param  uint8_t ucAddr，块地址
  * @param  pData，写入的数据，16字节
  * @retval 状态值= MI_OK，成功
  */
char PcdWrite ( uint8_t ucAddr, uint8_t * pData )
{
  char cStatus;
  uint8_t uc, ucComMF522Buf [ MAXRLEN ];
  uint32_t ulLen;
   
  
  ucComMF522Buf [ 0 ] = PICC_WRITE;
  ucComMF522Buf [ 1 ] = ucAddr;

  CalulateCRC ( ucComMF522Buf, 2, & ucComMF522Buf [ 2 ] );

  cStatus = PcdComMF522 ( PCD_TRANSCEIVE,
                          ucComMF522Buf,
                          4, 
                          ucComMF522Buf,
                          & ulLen );

  if ( ( cStatus != MI_OK ) || ( ulLen != 4 ) || 
         ( ( ucComMF522Buf [ 0 ] & 0x0F ) != 0x0A ) )
    cStatus = MI_ERR;   
      
  if ( cStatus == MI_OK )
  {
    //memcpy(ucComMF522Buf, pData, 16);
    for ( uc = 0; uc < 16; uc ++ )
      ucComMF522Buf [ uc ] = * ( pData + uc );  
    
    CalulateCRC ( ucComMF522Buf, 16, & ucComMF522Buf [ 16 ] );

    cStatus = PcdComMF522 ( PCD_TRANSCEIVE,
                           ucComMF522Buf, 
                           18, 
                           ucComMF522Buf,
                           & ulLen );
    
    if ( ( cStatus != MI_OK ) || ( ulLen != 4 ) || 
         ( ( ucComMF522Buf [ 0 ] & 0x0F ) != 0x0A ) )
      cStatus = MI_ERR;   
    
  } 	
  return cStatus;		
}


/**
  * @brief  读取M1卡数据
  * @param  ucAddr，块地址
  * @param  pData，读出的数据，16字节
  * @retval 状态值= MI_OK，成功
  */
char PcdRead ( uint8_t ucAddr, uint8_t * pData )
{
  char cStatus;
  uint8_t uc, ucComMF522Buf [ MAXRLEN ]; 
  uint32_t ulLen;
  
  ucComMF522Buf [ 0 ] = PICC_READ;
  ucComMF522Buf [ 1 ] = ucAddr;

  CalulateCRC ( ucComMF522Buf, 2, & ucComMF522Buf [ 2 ] );
 
  cStatus = PcdComMF522 ( PCD_TRANSCEIVE,
                          ucComMF522Buf,
                          4, 
                          ucComMF522Buf,
                          & ulLen );

  if ( ( cStatus == MI_OK ) && ( ulLen == 0x90 ) )
  {
    for ( uc = 0; uc < 16; uc ++ )
      * ( pData + uc ) = ucComMF522Buf [ uc ];   
  }
  
  else
    cStatus = MI_ERR;   
   
  return cStatus;		
}


/**
  * @brief  命令卡片进入休眠状态
  * @param  无
  * @retval 状态值= MI_OK，成功
  */
char PcdHalt( void )
{
	uint8_t ucComMF522Buf [ MAXRLEN ]; 
	uint32_t  ulLen;
  

  ucComMF522Buf [ 0 ] = PICC_HALT;
  ucComMF522Buf [ 1 ] = 0;
	
  CalulateCRC ( ucComMF522Buf, 2, & ucComMF522Buf [ 2 ] );
 	PcdComMF522 ( PCD_TRANSCEIVE,
                ucComMF522Buf,
                4, 
                ucComMF522Buf, 
                & ulLen );

  return MI_OK;	
}

/////////////////////////////////////////////////////////////////////
//功    能：写入钱包金额
//参数说明: ucAddr[IN]：块地址
//          pData：写入的金额
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char WriteAmount( uint8_t ucAddr, uint32_t pData )
{
	char status;
	uint8_t ucComMF522Buf[16];
	ucComMF522Buf[0] = (pData&((uint32_t)0x000000ff));
	ucComMF522Buf[1] = (pData&((uint32_t)0x0000ff00))>>8;
	ucComMF522Buf[2] = (pData&((uint32_t)0x00ff0000))>>16;
	ucComMF522Buf[3] = (pData&((uint32_t)0xff000000))>>24;	
	
	ucComMF522Buf[4] = ~(pData&((uint32_t)0x000000ff));
	ucComMF522Buf[5] = ~(pData&((uint32_t)0x0000ff00))>>8;
	ucComMF522Buf[6] = ~(pData&((uint32_t)0x00ff0000))>>16;
	ucComMF522Buf[7] = ~(pData&((uint32_t)0xff000000))>>24;	
	
	ucComMF522Buf[8] = (pData&((uint32_t)0x000000ff));
	ucComMF522Buf[9] = (pData&((uint32_t)0x0000ff00))>>8;
	ucComMF522Buf[10] = (pData&((uint32_t)0x00ff0000))>>16;
	ucComMF522Buf[11] = (pData&((uint32_t)0xff000000))>>24;	
	
	ucComMF522Buf[12] = ucAddr;
	ucComMF522Buf[13] = ~ucAddr;
	ucComMF522Buf[14] = ucAddr;
	ucComMF522Buf[15] = ~ucAddr;
  status = PcdWrite(ucAddr,ucComMF522Buf);
	return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取钱包金额
//参数说明: ucAddr[IN]：块地址
//          *pData：读出的金额
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char ReadAmount( uint8_t ucAddr, uint32_t *pData )
{
	
	char status = MI_ERR;
	uint8_t j;
	uint8_t ucComMF522Buf[16];
  status = PcdRead(ucAddr,ucComMF522Buf);
	if(status != MI_OK)
		return status;
	for(j=0;j<4;j++)
	{
		if((ucComMF522Buf[j] != ucComMF522Buf[j+8]) && (ucComMF522Buf[j] != ~ucComMF522Buf[j+4]))//验证一下是不是钱包的数据
		break;
	}
	if(j == 4)
	{
		  status = MI_OK;
			*pData = ucComMF522Buf[0] + (ucComMF522Buf[1]<<8) + (ucComMF522Buf[2]<<16) + (ucComMF522Buf[3]<<24);
	}
	else
	{
		status = MI_ERR;
		*pData = 0;
	}
  return status;	
}

static  uint32_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  EEPROM_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  
  return 0;
}
