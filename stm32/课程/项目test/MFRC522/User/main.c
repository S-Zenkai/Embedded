#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_mfrc522.h"
#include "bsp_SisTick.h"
#if 1

//uint8_t SectorKey[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//������Կ
int main(void)
{
	uint8_t SectorKey[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//������Կ
	uint8_t UIDBuff[4];
//	uint8_t WriteBuff[2]={0,100};
//	uint8_t ReadBuff[2];
	uint8_t WriteBuff=100;
	uint8_t ReadBuff;
	USART_Config();
	SisTick_Init();
	SPI_MFRC522_Init();
	
	PcdReset ();
  
  /*���ù�����ʽ*/
	M500PcdConfigISOType ( 'A' );
	
	while(1)
	{
		if(PcdRequest(PICC_REQIDL, UIDBuff)==MI_OK)//Ѱ��
		{
			if(PcdAnticoll(UIDBuff)==MI_OK)//����ײ
			{
				if(PcdSelect(UIDBuff)==MI_OK)//ѡ��
				{
					PcdAuthState(PICC_AUTHENT1A,0x04,SectorKey,UIDBuff);//��֤����
					PcdWrite(0x04, &WriteBuff);
					PcdRead ( 0x04, &ReadBuff );
//					WriteAmount(0x11,WriteBuff); //д����
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
  * @brief  IC���Ժ���
  * @param  ��
  * @retval ��
  */
uint8_t KeyValue[]={0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // ��A��Կ
void IC_test ( void )
{
	uint32_t writeValue = 100;
	uint32_t readValue;
	char cStr [ 30 ];
  uint8_t ucArray_ID [ 4 ];    /*�Ⱥ���IC�������ͺ�UID(IC�����к�)*/                                                                                         
	uint8_t ucStatusReturn;      /*����״̬*/                                                                                           
  while ( 1 )
  {    
    /*Ѱ��*/
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQIDL, ucArray_ID ) ) != MI_OK )  
       /*��ʧ���ٴ�Ѱ��*/
			ucStatusReturn = PcdRequest ( PICC_REQIDL, ucArray_ID );		                                                

		if ( ucStatusReturn == MI_OK  )
		{
      /*����ײ�����ж��ſ������д��������Χʱ������ͻ���ƻ������ѡ��һ�Ž��в�����*/
			if ( PcdAnticoll ( ucArray_ID ) == MI_OK )                                                                   
			{
				PcdSelect(ucArray_ID);			
		
				PcdAuthState( PICC_AUTHENT1A, 0x11, KeyValue, ucArray_ID );//У������ 
        WriteAmount(0x11,writeValue); //д����
        if(ReadAmount(0x11,&readValue) == MI_OK)	//��ȡ���
				{		
					writeValue +=100;
				  sprintf ( cStr, "The Card ID is: %02X%02X%02X%02X",ucArray_ID [0], ucArray_ID [1], ucArray_ID [2],ucArray_ID [3] );
					printf ( "%s\r\n",cStr );  //��ӡ��ƬID
					
					printf ("���Ϊ��%d\r\n",readValue);
					sprintf ( cStr, "TThe residual amount: %d", readValue);				 										 	         
          PcdHalt();
				}				
			}				
		}		
		    
  }	
}
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main ( void )
{
  /*�δ�ʱ�ӳ�ʼ��*/
	SisTick_Init();   
	
  /*USART1 ����ģʽΪ 115200 8-N-1���жϽ���*/
	USART_Config();   
	
  /*RC522ģ����������ĳ�ʼ������*/
	SPI_MFRC522_Init();     
	
    
		
	printf ( "WF-RC522 Test\n" );
	
	PcdReset ();
  
  /*���ù�����ʽ*/
	M500PcdConfigISOType ( 'A' );
	
  while ( 1 )
  {
    /*IC�����*/
    IC_test ();                	
  }	    
}

#endif


