#include "tim.h"

/*PI0   ------> TIM5_CH4   A  挡板右
  PH12  ------> TIM5_CH3   B  挡板左
	
	PH11 -------> TIM5_CH2   C  姿态右
	PH10 -------> TIM5_CH1   D  云台1PIT
	
	PD15  ------> TIM4_CH4   E  救援右
  PD14  ------> TIM4_CH3   F  救援左
	
	PD13  ------> TIM4_CH2   G  姿态左
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
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //预分频系数
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //时钟分频因子
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //小于CCR为有效值
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //有效电平为高电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //输出比较值CCR	
  
  TIM_OC4Init(TIM5,&TIM_OCInitStructure);
  TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable); //输出比较 4 预装载使能 OC4PE
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);               //自动重载预装载使能 ARPE
  
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
  
  TIM_TimeBaseInitStructure.TIM_Period        = per;                //重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;                //预分频系数
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //时钟分频因子
  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //小于CCR为有效值
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;      //有效电平为高电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能  
  TIM_OCInitStructure.TIM_Pulse       = 1000;                     //输出比较值CCR	
  
  TIM_OC4Init(TIM4,&TIM_OCInitStructure);
  TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //输出比较 4 预装载使能 OC4PE
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); 
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);               //自动重载预装载使能 ARPE
  
  TIM_Cmd(TIM4,ENABLE);
}



//void TIM4_ENCODER_Init(void)                      
//{ 
//    GPIO_InitTypeDef GPIO_InitStruct;            /*GPIO*/
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct; /*时基*/
//    TIM_ICInitTypeDef TIM_ICInitStruct;          /*输入通道*/
//    
//    /*GPIO初始化*/    
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); /*使能GPIO时钟 AHB1*/                    
//    GPIO_StructInit(&GPIO_InitStruct);        
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; 
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        /*复用功能*/
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  /*速度100MHz*/
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        
//    GPIO_Init(GPIOD, &GPIO_InitStruct); 
//    
//    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 
//    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); 

//    /*时基初始化*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   /*使能定时器时钟 APB1*/
//    TIM_DeInit(TIM4);  
//    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);    
//    TIM_TimeBaseStruct.TIM_Prescaler = ENCODER_TIM_PSC;       /*预分频 */        
//    TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;       /*周期(重装载值)*/
//    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;      
//    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;  /*连续向上计数模式*/  
//    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct); 

//    /*编码器模式配置：同时捕获通道1与通道2(即4倍频)，极性均为Rising*/
//    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
//    TIM_ICStructInit(&TIM_ICInitStruct);        
//    TIM_ICInitStruct.TIM_ICFilter = 0;   /*输入通道的滤波参数*/
//    TIM_ICInit(TIM4, &TIM_ICInitStruct); /*输入通道初始化*/
//    TIM_SetCounter(TIM4, CNT_INIT);      /*CNT设初值*/
//    TIM_ClearFlag(TIM4,TIM_IT_Update);   /*中断标志清0*/
//    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); /*中断使能*/
//    TIM_Cmd(TIM4,ENABLE);                /*使能CR寄存器*/
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


