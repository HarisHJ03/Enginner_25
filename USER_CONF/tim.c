#include "tim.h"

/*PI0   ------> TIM5_CH4   A  ������
  PH12  ------> TIM5_CH3   B  ������
	
	PH11 -------> TIM5_CH2   C  ��̬��
	PH10 -------> TIM5_CH1   D  ��̨1PIT
	
	PD15  ------> TIM4_CH4   E  ��Ԯ��
  PD14  ------> TIM4_CH3   F  ��Ԯ��
	
	PD13  ------> TIM4_CH2   G  ��̬��
	*/
	
void TIM5_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  
  GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //��װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //ʱ�ӷ�Ƶ����
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //С��CCRΪ��Чֵ
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //��Ч��ƽΪ�ߵ�ƽ
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //�Ƚ����ʹ��  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //����Ƚ�ֵCCR	
  
  TIM_OC4Init(TIM5,&TIM_OCInitStructure);
  TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable); //����Ƚ� 4 Ԥװ��ʹ�� OC4PE
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);               //�Զ�����Ԥװ��ʹ�� ARPE
  
  TIM_Cmd(TIM5,ENABLE);
}
void TIM4_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //��װ��ֵ
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //ʱ�ӷ�Ƶ����
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //С��CCRΪ��Чֵ
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //��Ч��ƽΪ�ߵ�ƽ
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //�Ƚ����ʹ��  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //����Ƚ�ֵCCR	
  
  TIM_OC4Init(TIM4,&TIM_OCInitStructure);
  TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //����Ƚ� 4 Ԥװ��ʹ�� OC4PE
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);               //�Զ�����Ԥװ��ʹ�� ARPE
  
  TIM_Cmd(TIM4,ENABLE);
}



//void TIM4_ENCODER_Init(void)                      
//{ 
//    GPIO_InitTypeDef GPIO_InitStruct;            /*GPIO*/
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct; /*ʱ��*/
//    TIM_ICInitTypeDef TIM_ICInitStruct;          /*����ͨ��*/
//    
//    /*GPIO��ʼ��*/    
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); /*ʹ��GPIOʱ�� AHB1*/                    
//    GPIO_StructInit(&GPIO_InitStruct);        
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; 
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        /*���ù���*/
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  /*�ٶ�100MHz*/
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        
//    GPIO_Init(GPIOD, &GPIO_InitStruct); 
//    
//    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 
//    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 

//    /*ʱ����ʼ��*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   /*ʹ�ܶ�ʱ��ʱ�� APB1*/
//    TIM_DeInit(TIM4);  
//    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);    
//    TIM_TimeBaseStruct.TIM_Prescaler = ENCODER_TIM_PSC;       /*Ԥ��Ƶ */        
//    TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;       /*����(��װ��ֵ)*/
//    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;      
//    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;  /*�������ϼ���ģʽ*/  
//    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct); 

//    /*������ģʽ���ã�ͬʱ����ͨ��1��ͨ��2(��4��Ƶ)�����Ծ�ΪRising*/
//    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//    TIM_ICStructInit(&TIM_ICInitStruct);        
//    TIM_ICInitStruct.TIM_ICFilter = 0;   /*����ͨ�����˲�����*/
//    TIM_ICInit(TIM4, &TIM_ICInitStruct); /*����ͨ����ʼ��*/
//    TIM_SetCounter(TIM4, CNT_INIT);      /*CNT���ֵ*/
//    TIM_ClearFlag(TIM4,TIM_IT_Update);   /*�жϱ�־��0*/
//    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); /*�ж�ʹ��*/
//    TIM_Cmd(TIM4,ENABLE);                /*ʹ��CR�Ĵ���*/
//} 
/*PWM*/
void TIM2_DEVICE(int per,int psc)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 1000;
  
  TIM_OC1Init(TIM2,&TIM_OCInitStructure);
  TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
 	TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	
  TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2,ENABLE);
  
  TIM_Cmd(TIM2,ENABLE);
}


