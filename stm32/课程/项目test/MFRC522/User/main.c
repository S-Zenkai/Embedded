#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_mfrc522.h"
#include "bsp_SisTick.h"
#if 1

//uint8_t SectorKey[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//扇区密钥
int main(void)
{
	uint8_t SectorKey[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//扇区密钥
	uint8_t UIDBuff[4];
//	uint8_t WriteBuff[2]={0,100};
//	uint8_t ReadBuff[2];
	uint8_t WriteBuff=100;
	uint8_t ReadBuff;
	USART_Config();
	SisTick_Init();
	SPI_MFRC522_Init();
	
	PcdReset ();
  
  /*设置工作方式*/
	M500PcdConfigISOType ( 'A' );
	
	while(1)
	{
		if(PcdRequest(PICC_REQIDL, UIDBuff)==MI_OK)//寻卡
		{
			if(PcdAnticoll(UIDBuff)==MI_OK)//防冲撞
			{
				if(PcdSelect(UIDBuff)==MI_OK)//选卡
				{
					PcdAuthState(PICC_AUTHENT1A,0x04,SectorKey,UIDBuff);//验证密码
					PcdWrite(0x04, &WriteBuff);
					PcdRead ( 0x04, &ReadBuff );
//					WriteAmount(0x11,WriteBuff); //写入金额
//					ReadAmount(0x11,&ReadBuff);
//					printf("%d%d\n",ReadBuff[0],ReadBuff[1]);
					printf("%d\n",ReadBuff);
				}
			}
		}
	}
}

#elif 0
/**
  * @brief  IC测试函数
  * @param  无
  * @retval 无
  */
uint8_t KeyValue[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // 卡A密钥
void IC_test ( void )
{
	uint32_t writeValue = 100;
	uint32_t readValue;
	char cStr [ 30 ];
  uint8_t ucArray_ID [ 4 ];    /*先后存放IC卡的类型和UID(IC卡序列号)*/                                                                                         
	uint8_t ucStatusReturn;      /*返回状态*/                                                                                           
  while ( 1 )
  {    
    /*寻卡*/
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQIDL, ucArray_ID ) ) != MI_OK )  
       /*若失败再次寻卡*/
			ucStatusReturn = PcdRequest ( PICC_REQIDL, ucArray_ID );		                                                

		if ( ucStatusReturn == MI_OK  )
		{
      /*防冲撞（当有多张卡进入读写器操作范围时，防冲突机制会从其中选择一张进行操作）*/
			if ( PcdAnticoll ( ucArray_ID ) == MI_OK )                                                                   
			{
				PcdSelect(ucArray_ID);			
		
				PcdAuthState( PICC_AUTHENT1A, 0x11, KeyValue, ucArray_ID );//校验密码 
        WriteAmount(0x11,writeValue); //写入金额
        if(ReadAmount(0x11,&readValue) == MI_OK)	//读取金额
				{		
					writeValue +=100;
				  sprintf ( cStr, "The Card ID is: %02X%02X%02X%02X",ucArray_ID [0], ucArray_ID [1], ucArray_ID [2],ucArray_ID [3] );
					printf ( "%s\r\n",cStr );  //打印卡片ID
					
					printf ("余额为：%d\r\n",readValue);
					sprintf ( cStr, "TThe residual amount: %d", readValue);				 										 	         
          PcdHalt();
				}				
			}				
		}		
		    
  }	
}
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main ( void )
{
  /*滴答时钟初始化*/
	SisTick_Init();   
	
  /*USART1 配置模式为 115200 8-N-1，中断接收*/
	USART_Config();   
	
  /*RC522模块所需外设的初始化配置*/
	SPI_MFRC522_Init();     
	
    
		
	printf ( "WF-RC522 Test\n" );
	
	PcdReset ();
  
  /*设置工作方式*/
	M500PcdConfigISOType ( 'A' );
	
  while ( 1 )
  {
    /*IC卡检测*/
    IC_test ();                	
  }	    
}

#endif


