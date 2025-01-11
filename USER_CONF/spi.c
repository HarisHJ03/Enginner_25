#include "spi.h"

#include "stm32f4xx.h"
//void SPI_DEVICE(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	SPI_InitTypeDef SPI_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,ENABLE);
//	
//  GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5);
//	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5);
//	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5);
//  
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOF,&GPIO_InitStructure);
//	
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_CRCPolynomial = 10;
//	SPI_Init(SPI5,&SPI_InitStructure);
//	
//	SPI_Cmd(SPI5,ENABLE);
//	
//}


void MySPI_W_SS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_0, (BitAction)BitValue);		//??BitValue,??SS?????
}


void MySPI_W_SCK(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_1, (BitAction)BitValue);		//??BitValue,??SCK?????
}

/*accel�Ķ�д*/

uint8_t MySPI_R_MISO_ACCEL(void)
{
	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);			//??MISO?????
}

void MySPI_W_MOSI_ACCEL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_4, (BitAction)BitValue);		//??BitValue,??MOSI?????,BitValue????0?1???
}


/*GYRO�Ķ�д*/
uint8_t MySPI_R_MISO_GYRO(void)
{
	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);			//??MISO?????
}

void MySPI_W_MOSI_GYRO(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_5, (BitAction)BitValue);		//??BitValue,??MOSI?????,BitValue????0?1???
}



void SPI_DEVICE(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//??GPIOA???
	
	/*0Ϊss  1ΪSCL 2Ϊaccel_MOSI 3ΪGyro_MOSI*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	MySPI_W_SS(1);											
	MySPI_W_SCK(0);											
}


void MySPI_Start(void)
{
	MySPI_W_SS(0);				//??SS,????
}


void MySPI_Stop(void)
{
	MySPI_W_SS(1);				//??SS,????
}


uint8_t MySPI_SwapByte_ACCEL(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;					//???????,????0x00,???????0x00,?????
	
	for (i = 0; i < 8; i ++)						//??8?,?????????
	{
		MySPI_W_MOSI_ACCEL(ByteSend & (0x80 >> i));		//?????????ByteSend???????????MOSI?
		MySPI_W_SCK(1);								//??SCK,???????
		if (MySPI_R_MISO_ACCEL() == 1){ByteReceive |= (0x80 >> i);}	//??MISO??,????Byte??
																//?MISO?1?,???????1,?MISO?0?,????,?????????0
		MySPI_W_SCK(0);								//??SCK,???????
	}
	
	return ByteReceive;								//????????????
}

uint8_t MySPI_SwapByte_GYRO(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;					//???????,????0x00,???????0x00,?????
	
	for (i = 0; i < 8; i ++)						//??8?,?????????
	{
		MySPI_W_MOSI_GYRO(ByteSend & (0x80 >> i));		//?????????ByteSend???????????MOSI?
		MySPI_W_SCK(1);								//??SCK,???????
		if (MySPI_R_MISO_GYRO() == 1){ByteReceive |= (0x80 >> i);}	//??MISO??,????Byte??
																//?MISO?1?,???????1,?MISO?0?,????,?????????0
		MySPI_W_SCK(0);								//??SCK,???????
	}
	
	return ByteReceive;								//????????????
}


 void BMI088_read_muli_reg_ACCLE(uint8_t reg, uint8_t *buf, uint8_t len)
{
	
  MySPI_SwapByte_ACCEL(reg | 0x80);
	while (len != 0)
	{

		*buf = MySPI_SwapByte_ACCEL(0x55);
		buf++;
		len--;
	}
}

 void BMI088_read_muli_reg_GORY(uint8_t reg, uint8_t *buf, uint8_t len)
{
	//BMI088_read_write_byte(reg | 0x80);
   MySPI_SwapByte_GYRO(reg | 0x80);
	while (len != 0)
	{

		*buf = MySPI_SwapByte_GYRO(0x55);
		
		buf++;
		len--;
	}
}
