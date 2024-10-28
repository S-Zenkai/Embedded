/**
 ****************************************************************************************************
 * @file        bsp_oled.c
 * @author      
 * @version     
 * @date        2024-09-13
 * @brief       oled��ʾʵ�飬ʹ��IICͨ��
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:stm32f103vet6��0.96��oled��ʾ����SSD1306оƬ��
 *
 ****************************************************************************************************
 */

#include "bsp_oled.h"
#include "./I2C_Software/bsp_i2c.h"
#include "./OLED/bsp_oleddata.h"
#include "bsp_delay.h"
#include <math.h>

/** 
 *  ���귽��
  0����������������Column(Y)(+)(0~127)
  |
  |
  |
  |
	|
	|Page(X)(+)(0~7)  
 */


/** 
 *  Height,Width���壺
 *  Height��ÿһ�����ص����1���߶ȣ����ȣ�
 */

/** 
 *  �Դ�����
 *  ��ֱ����GDDRAM��д���ݣ�����ͨ����д�Դ����飬��ͨ�����Դ�����д��GDDRAM��
 *  �ŵ㣺��ʵ�ֶ�GDDRAM���ݵĶ�ȡ������ssd1306ʹ��IIC���ܽ��ж����������������ʹ��IIC��ȡ��
 *  GPU��SRAM�ٶȸ���
 */
uint8_t OLED_DisplayBuff[8][128];

void OLED_IIC_init(void)
{
	//��ʼ��IIC SDA��SCL���GPIO
	i2c_init();
	delay_ms(100);
	/*д��һϵ�е������OLED���г�ʼ������*/
	SendCommand(0xAE);	//������ʾ����/�رգ�0xAE�رգ�0xAF����
	
	SendCommand(0xD5);	//������ʾʱ�ӷ�Ƶ��/����Ƶ��
	SendCommand(0x80);	//0x00~0xFF
	
	SendCommand(0xA8);	//���ö�·������
	SendCommand(0x3F);	//0x0E~0x3F
	
	SendCommand(0xD3);	//������ʾƫ��
	SendCommand(0x00);	//0x00~0x7F
	
	SendCommand(0x40);	//������ʾ��ʼ�У�0x40~0x7F
	
	SendCommand(0xA1);	//�������ҷ���0xA1������0xA0���ҷ���
	
	SendCommand(0xC8);	//�������·���0xC8������0xC0���·���

	SendCommand(0xDA);	//����COM����Ӳ������
	SendCommand(0x12);
	
	SendCommand(0x81);	//���öԱȶ�
	SendCommand(0xCF);	//0x00~0xFF

	SendCommand(0xD9);	//����Ԥ�������
	SendCommand(0xF1);

	SendCommand(0xDB);	//����VCOMHȡ��ѡ�񼶱�
	SendCommand(0x30);

	SendCommand(0xA4);	//����������ʾ��/�ر�

	SendCommand(0xA6);	//��������/��ɫ��ʾ��0xA6������0xA7��ɫ

	SendCommand(0x8D);	//���ó���
	SendCommand(0x14);

	SendCommand(0xAF);	//������ʾ
	
	OLED_Clear();       //���Դ�����ȫ������
	OLED_Updata();      //����GDDRAM
}

/**
 *  @brief   ��ssd1306���������ź� 
 *  @param   Cmd ssd1306�������ֽڣ���ѡ0x00~0xFF
 *  @retval  ��
 */
void SendCommand(uint8_t Cmd)
{
	I2C_Start();
	I2C_WriteByte(OLED_Addr);
	I2C_WaitAck();
	I2C_WriteByte(CmdControlByte);//���÷�����ģʽ��Co=0��,Control byte�ɿ�����һ�����͵������ݻ�������
	I2C_WaitAck();
	I2C_WriteByte(Cmd);
	I2C_WaitAck();
	I2C_Stop();
}


/**
 *  @brief   ��ssd1306����һ���ֽ� 
 *  @param   DataBuff Ҫ���͵����ݣ��ֽڣ�
 *           num ����������
 *  @retval  ��
 */
void OLED_SendByte(uint8_t DataBuff)
{
	I2C_Start();
	I2C_WriteByte(OLED_Addr);
	I2C_WaitAck();
	I2C_WriteByte(DataControlByte);//���÷�����ģʽ��Co=0��,Control byte�ɿ�����һ�����͵������ݻ�������
	I2C_WaitAck();
	I2C_WriteByte(DataBuff);
	I2C_WaitAck();
	I2C_Stop();
}

/**
 *  @brief   ��ssd1306���Ͷ���ֽ� 
 *  @param   DataBuff Ҫ���͵����ݣ��ֽڣ�
 *           num ����������
 *  @retval  ��
 */
void SendBytes(uint8_t* DataBuff,uint16_t num)
{
	uint16_t i;
	I2C_Start();
	I2C_WriteByte(OLED_Addr);
	I2C_WaitAck();
	I2C_WriteByte(DataControlByte);//���÷�����ģʽ��Co=0��,Control byte�ɿ�����һ�����͵������ݻ�������
	I2C_WaitAck();
	for(i=0;i<num;i++)
	{
		I2C_WriteByte(*(DataBuff+i));
		I2C_WaitAck();
	}
	I2C_Stop();
}

/**
 *  @brief   ����д�����ݵ���ʼ����
 *  @param   Page ҳ��ַ����Χ��0~7
 *           Column �е�ַ����Χ��0~127
 *  @retval  ��
 */
void SetCoordinate(uint8_t Page,uint8_t Column)
{
	SendCommand(0xB0|Page);//����ҳ��ַ
	SendCommand(0x10|((Column&0xF0)>>4));//�����е�ַ����λ
	SendCommand(0x00|(Column&0x0F));//�����е�ַ����λ
}

/**
 *  @brief   ���Դ�����ȫ������
 *  @param   ��
 *  @retval  ��
 */
void OLED_Clear(void)
{
	uint8_t i=0;
	uint8_t j=0;
	for(i=0;i<8;i++)
	{
		for(j=0;j<128;j++)
		{
			OLED_DisplayBuff[i][j]=0;
		}
	}
}

/**
 *  @brief   ���Դ����鲿������
 *  @param   X �Դ�������ʼ�У���Χ0~65535
 *           Height Ҫ���������߶ȣ�ÿ�����ص��ʾһ���߶ȣ���������
 *           Y �Դ�������ʼ�У���Χ0~65535
 *           Width Ҫ������������
 *  @retval  ��
 */
void OLED_ClearPart(uint16_t X,uint16_t Y,uint16_t Height,uint16_t Width)
{
	uint8_t i=0;
	uint8_t j=0;
	for(i=X;i<X+Height;i++)//+1��Ϊ�˶�X+Heightλ���в���
	{
		for(j=Y;j<Y+Width;j++)
		{
			if(i<64&&j<128)
			{
				OLED_DisplayBuff[i/8][j] &= !(1<<(i%8));//ֻ���λ���Գ�����Ĳ���
			}
		}
	}
}

/**
 *  @brief   ���Դ�����ȫ��ȡ��
 *  @param   ��
 *  @retval  ��
 */
void OLED_Invert(void)
{
	uint8_t i=0;
	uint8_t j=0;
	for(i=0;i<8;i++)
	{
		for(j=0;j<128;j++)
		{
			OLED_DisplayBuff[i][j] ^=0xFF;
		}
	}
}

/**
 *  @brief   ���Դ����鲿��ȡ��
 *  @param   X �Դ�������ʼ�У���Χ0~65535
 *           Height Ҫ���������߶�
 *           Y �Դ�������ʼ�У���Χ0~65535
 *           Width Ҫ������������
 *  @retval  ��
 */
void OLED_InvertPart(uint16_t X,uint16_t Y,uint16_t Height,uint16_t Width)
{
	uint8_t i=0;
	uint8_t j=0;
	for(i=X;i<X+Height;i++)
	{
		for(j=Y;j<Y+Width;j++)
		{
			if(i<64&&j<128)
			{
				OLED_DisplayBuff[i/8][j] ^= (0x01<<(i%8));//ֻȡ��λ���Դ�����Ĳ���
			}
		}
	}
}

/**
 *  @brief   ���Դ�����ȫ�����µ�ssd1306��GDDRAM
 *  @param   ��
 *  @retval  ��
 */
void OLED_Updata(void)
{
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
		SetCoordinate(i,0);
		SendBytes(OLED_DisplayBuff[i],128);
	}
}


/**
 *  @brief   ���Դ����鲿�ָ��µ�ssd1306��GDDRAM
 *  @param   X �Դ�������ʼ�У���Χ0~65535
 *           Height Ҫ���������߶�,��Χ0~65535
 *           Y �Դ�������ʼ�У���Χ0~65535
 *           Width Ҫ������������,��Χ0~65535
 *  @retval  ��
 */
void OLED_UpdataPart(int16_t X,int16_t Y,int16_t Height,int16_t Width)
{
	int16_t i=0;
	int16_t j=0;
	int16_t Page0=X/8;
	int16_t Page1=(X+Height-1)/8;
	if(X<0)
	{
		Page0=Page0-1;
		Page1=Page1-1;
	}
	for(i=Page0;i<Page1+1;i++)
	{
			if(i>=0&&i<8&&j>=0&&j<128)
			{
				SetCoordinate(i,Y);
				SendBytes(&OLED_DisplayBuff[i][Y],Width);
			}
	}
}


/**
  *  @brief   ��ʾͼ��
  *  @note    ����Ҫ���ø��º���
  *  @note    �����ssd1306���ܶ���ֻ����ҳд������⣺ͨ�������Դ����飬�����˶�ȡssd1306��
  *           ���ż�����ʼҳƫ�ƣ�ͨ�������ֻ�޸ı�ҳ��ʼ��ַ���λ��8λʣ�����ݷ�����һҳ��
  *           �Դ�������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Height Ҫ���������߶�,��Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *           Width Ҫ������������,��Χ0~65535
  *  @retval  ��
  */
void OLED_DisplayImage(uint16_t X,uint16_t Y,uint16_t Height,uint16_t Width,const uint8_t* ImageBuff)
{
	uint16_t i,j;
	uint16_t Page0;
	OLED_ClearPart(X,Y,Height,Width);
	Page0=X/8;

	for(i=0;i<(Height-1)/8+1;i++)//���ﲻӦ���ǽ���ҳ����ʼҳ����Ϊfor�ڲ�����������ҳ
	{
		for(j=0;j<Width;j++)
		{
			if((Y+j)>=0&&(Y+j)<128)//ֻ��ʾ��������
			{
				if(Page0+i >= 0 && Page0+i < 8)//ֻ��ʾҳ������
					OLED_DisplayBuff[Page0+i][Y+j] |= ImageBuff[i*Width+j]<<(X%8);//���ﴦ����ssd1306ֻ����ҳд������⣬������˵��
				if(Page0+i+1 >= 0 && Page0+i+1 < 8)//ֻ��ʾҳ������
					OLED_DisplayBuff[Page0+i+1][Y+j] |= ImageBuff[i*Width+j]>>(8-X%8);
			}
		}
	}
}


/**
  *  @brief   ��ʾһ���ַ�
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayChar(uint16_t X,uint16_t Y,char Char,uint8_t FontSize)
{
	
	if(FontSize==H16W8)
	{
		OLED_DisplayImage(X,Y,16,8,OLED_F8x16[Char-' ']);
		OLED_UpdataPart(X,Y,16,8);
	}
	if(FontSize==H8W6)
	{
		OLED_DisplayImage(X,Y,8,6,OLED_F6x8[Char-' ']);
		OLED_UpdataPart(X,Y,8,6);
	}
}

/**
  *  @brief   ��ʾ�ַ���
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayCharString(uint16_t X,uint16_t Y,char* String,uint8_t FontSize)
{
	uint8_t i;
	//��Ϊ���ﴫ�����ַ��������Բ���֪�������С��ֱ���ж�\n����
	for(i=0;String[i]!='\0';i++)//����Ӧ��ѧϰһ�£�ָ�봫������һ����Ҫ�����ã�Ҳ������������
	{
		OLED_DisplayChar(X,Y+8*i,String[i],FontSize);
	}
	if(FontSize==H16W8)
	{
		//��Ϊ���ﴫ�����ַ��������Բ���֪�������С��ֱ���ж�\n����
		for(i=0;String[i]!='\0';i++)//����Ӧ��ѧϰһ�£�ָ�봫������һ����Ҫ�����ã�Ҳ������������
		{
			OLED_DisplayChar(X,Y+8*i,String[i],FontSize);
		}
	}
	if(FontSize==H8W6)
	{
		//��Ϊ���ﴫ�����ַ��������Բ���֪�������С��ֱ���ж�\n����
		for(i=0;String[i]!='\0';i++)//����Ӧ��ѧϰһ�£�ָ�봫������һ����Ҫ�����ã�Ҳ������������
		{
			OLED_DisplayChar(X,Y+6*i,String[i],FontSize);
		}
	}
}

/**
  *  @brief   �η�����
  *  @note    ֻ�ܼ������η�����������󲻳�����-2147483648 �� 2147483647��
  *  @param   num �η��ף���Χ��-2147483648 �� 2147483647��
  *  @param   pow �η�����Χ�� 0 �� 65535
  *  @retval  num^pow
  */
static int32_t OLED_Pow(int32_t num,uint16_t pow)
{
	uint16_t i;
	int32_t ret=1;
	if(pow==0)
		return ret;
	for(i=0;i<pow;i++)
	{
		ret *= num;
	}
	return ret;
}

/**
  *  @brief   ��ʾ���֣�ʮ���ƣ���������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntNum������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayNumber(uint16_t X,uint16_t Y,uint32_t Num,uint8_t IntBit,uint8_t FontSize)
{
	uint8_t i;
	uint8_t tmp;//���Ҫ��ʾ���ֵ�ÿһλ
	for(i=0;i<IntBit;i++)
	{
		tmp=Num/OLED_Pow(10,IntBit-1-i);//ȡ��Num��ÿһλ
		Num=Num%OLED_Pow(10,IntBit-1-i);
		OLED_DisplayChar(X,Y+FontSize*i,tmp+'0',FontSize);
	}
}
	
/**
  *  @brief   ��ʾ���֣�ʮ���ƣ�������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntBit������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplaySignNumber(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize)
{
	uint8_t i;
	uint8_t tmp;//���Ҫ��ʾ���ֵ�ÿһλ
	if(Num>=0)
	{
		OLED_DisplayChar(X,Y,'+',FontSize);
	}
	if(Num<0)
	{
		OLED_DisplayChar(X,Y,'-',FontSize);
		Num=-Num;
	}
	for(i=0;i<IntBit;i++)
	{
		tmp=Num/OLED_Pow(10,IntBit-1-i);//ȡ��Num��ÿһλ
		
		Num=Num%OLED_Pow(10,IntBit-1-i);
		OLED_DisplayChar(X,Y+FontSize*(i+1),tmp+'0',FontSize);
	}
}

/**
  *  @brief   ��ʾ���֣�ʮ�����ƣ���������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntBit������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayHexNum(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize)
{
	uint8_t i;
	uint8_t tmp;//���Ҫ��ʾ���ֵ�ÿһλ
	OLED_DisplayChar(X,Y,'0',FontSize);
	OLED_DisplayChar(X,Y+FontSize*1,'X',FontSize);
	for(i=0;i<IntBit;i++)
	{
		tmp=(Num/OLED_Pow(16,IntBit-1-i))%16;//ȡ��Num��ÿһλ
		if(tmp<10)
		{
			OLED_DisplayChar(X,Y+FontSize*(i+2),tmp+'0',FontSize);
		}
		else
		{
			OLED_DisplayChar(X,Y+FontSize*(i+2),tmp-10+'A',FontSize);
		}
	}
}

/**
  *  @brief   ��ʾ���֣������ƣ���������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntBit������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayBinNum(uint16_t X,uint16_t Y,int32_t Num,uint8_t IntBit,uint8_t FontSize)
{
	uint8_t i;
	uint8_t tmp;//���Ҫ��ʾ���ֵ�ÿһλ
	OLED_DisplayChar(X,Y,'0',FontSize);
	OLED_DisplayChar(X,Y+FontSize*1,'X',FontSize);
	for(i=0;i<IntBit;i++)
	{
		tmp=(Num/OLED_Pow(2,IntBit-1-i))%2;//ȡ��Num��ÿһλ
    OLED_DisplayChar(X,Y+FontSize*i,tmp+'0',FontSize);
	}
	OLED_DisplayChar(X,Y+FontSize*i,'B',FontSize);
}


/**
  *  @brief   ��ʾ������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntNum����������λ��
  *  @param   DecNum��С������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayFloat(uint16_t X,uint16_t Y,double Num,uint8_t IntBit,uint8_t DecBit,uint8_t FontSize)
{
	int32_t Int;
	double Flt;
	uint8_t Pow;
	if(Num>=0)
	{
		OLED_DisplayChar(X,Y,'+',FontSize);
	}
	if(Num<0)
	{
		OLED_DisplayChar(X,Y,'-',FontSize);
		Num=-Num;
	}
	Int=Num;//�����������
	Num -= Int;//��ֹ����ĳ˷�����������
	Pow=OLED_Pow(10,DecBit);
	Flt=Num*Pow;
	Flt=round(Flt);//��������
//	Int += Flt / Pow;				//��������������˽�λ������Ҫ�ټӸ�����(���ƴ�Ĵ��룬��֪����ʲô��)
	OLED_DisplayNumber(X,Y+FontSize,Int,IntBit,FontSize);
	OLED_DisplayChar(X,Y+FontSize*(IntBit+1),'.',FontSize);
	OLED_DisplayNumber(X,Y+FontSize*(IntBit+2),Flt,DecBit,FontSize);
}


/**
  *  @brief   ��ʾ������
  *  @param   X �Դ�������ʼ�У���Χ0~65535
  *           Y �Դ�������ʼ�У���Χ0~65535
  *  @param   Num��Ҫ��ʾ������
  *  @param   IntNum����������λ��
  *  @param   DecNum��С������λ��
  *  @param   FontSize �����С������Ϊ�����ֵ
  *    @arg   H16W8�������16��7�����أ�
  *           H8W6 �������8��6�����أ�
  *  @retval  ��
  */
void OLED_DisplayChinese(uint16_t X, uint16_t Y, char* Buff)
{
	uint8_t pindex = 0;
	uint8_t i = 0;
	while (strcmp(OLED_CF16x16[pindex], "") == 0)
	{

	}
}





