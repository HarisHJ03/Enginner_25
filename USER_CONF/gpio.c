

#include "gpio.h"
#include "delay.h"
#include "bsp_flash.h"
#include "bsp_can.h"

#include "IO.h"
static void KEY_EXTI(void);

/**************************************************
GPIOG:PIN1~PIN8  led��
GPIOB:PIN2       key����
GPIOF:PIN5       imuƬѡ��

I1->GPIOF PIN1   ���ÿ���
J1->GPIOE PIN5	 ��б
K1->GPIOE PIN6   ����
L1->GPIOC PIN2	 ����
M1->GPIOC PIN3   �ϰ���

Q1->GPIOF PIN10		������ź�

GPIOI PIN0 ̧������ѹ��

***************************************************/


/****************2025����pin��******************************

GPIOC 0����5 ���spi








**********************************************/
void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	/*LED��*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
  GPIO_SetBits(GPIOG,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOG,GPIO_Pin_8);
	
	/*GPIOA*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	
  /*GPIOC*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//  
//  GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);
	
	/*��λ���ع���� PC5 O1*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
  
  //GPIO_SetBits(GPIOC,GPIO_Pin_5);
	
	/*��λ���ع���� PF10 Q1*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
  GPIO_SetBits(GPIOF,GPIO_Pin_10);
	
  /*GPIOE*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
  
  GPIO_SetBits(GPIOE,GPIO_Pin_5|GPIO_Pin_6);
  
  /*GPIOF*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
  GPIO_SetBits(GPIOF,GPIO_Pin_1);
  /*GPIOH   ������Ŀɿص�Դ���24V*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
//	GPIO_Init(GPIOH,&GPIO_InitStructure);
//  
//  GPIO_SetBits(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
	
  /*GPIOI   ������ʾ������IO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	/*��ȡIO���ã����ģʽ��*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	/*��ȡͷ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	/*��ȡβ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	/*����ͷ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	/*����β*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*����ͷ*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*����β*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	/*�ϰ�ͷ*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
	/*�ϰ�β*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	/*��ʾ���л�*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	  GPIO_SetBits(GPIOF,GPIO_Pin_0);

	
	/*IMUƬѡ��*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	/*KEY����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/*��ȡ��λ���أ�����ģʽ��*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
	
		/*̧����λ���أ�����ģʽ��*/
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	
	GPIO_InitStructure.GPIO_Pin = UPRISE_KEY1_PIN;
	GPIO_Init(UPRISE_KEY1_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UPRISE_KEY2_PIN;
	GPIO_Init(UPRISE_KEY2_PORT,&GPIO_InitStructure);
	
	KEY_EXTI();
}


/*****************************************************************
PB2:KEY�����ⲿ�ⲿ�ж�����
******************************************************************/
static void KEY_EXTI(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);			//ʹ���ⲿ�ж�Դʱ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);	//ӳ�䵽IO��
	
	//EXTI2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//EXTI2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
	EXTI_InitStructure.EXTI_Line=EXTI_Line2; 
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}


uint8_t key_state;//����״̬��־λ

//void EXTI2_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line2)==1)
//	{
//		delay_ms(10);
//		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)==Bit_SET)
//		{	
//			
//			cali_param.yaw_offset = moto_yaw.ecd;
//			cali_param.pitch_offset = moto_pit.ecd;
//      cali_param.cali_state = CALI_DONE;
//			key_state = BSP_FLASH_WRITE((uint8_t *)&cali_param,sizeof(cali_sys_t));
//			
//		}
//	}
//	EXTI_ClearITPendingBit(EXTI_Line2);    
//}
