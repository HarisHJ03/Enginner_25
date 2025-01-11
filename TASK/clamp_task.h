#ifndef _clamp_task_H
#define _clamp_task_H


#include "stm32f4xx.h"
#include "ramp.h"

//#define CLAMP_ON   	 	 GPIO_SetBits(GPIOF,GPIO_Pin_1); GPIO_SetBits(GPIOE,GPIO_Pin_5);
//#define PUMP_OFF   	 GPIO_ResetBits(GPIOF,GPIO_Pin_1);GPIO_ResetBits(GPIOE,GPIO_Pin_5);



typedef enum
{
  CLAMPING = 0,
  CLAMPED,
}clamp_flag_t;

/*第四代仅能夹取两箱*/
typedef enum
{
  INIT_BOX = 0, //并未夹取
  FRIST_BOX,    //当前已夹取了一箱  准备夹取第二箱
  SECOND_BOX,   //当前已夹取了两箱  准备夹取第三箱
  THIRD_BOX,    //当前已夹取了三箱  夹取完所有
}pick_box_t;

typedef enum
{
	CLAMP_ADJUST_UNCMD = 0,
  CLAMP_ADJUST_CMD ,
}clamp_adjust_t;

typedef enum
{
	ZERO = 0,
	ONE,
	TWO,
	THREE ,
	FOUR,
	FIVE,
}big_island_ore_numbering_t;//大资源岛矿石编号

typedef enum
{
	EXCGANHE_INIT = 0,
	MI_TO_PARA_POSITION,//打在中间但是执行对位代码
	MI_TO_ORE_ADJUST,       //打在中间但是执行储矿姿态调整
	UP_TO_REACTION,         //打在上面但是执行感应推落动作
	UP_TOERROR_CORRECTION,  //打在上面但是执行未能推落的补救

}exchange_sequence_t;//因为档位复用不同进入顺序 需要执行不同代码 所以添加此
//typedef struct
//{
//  int16_t offset_angle_left;
//  int16_t offset_angle_mid;
//  int16_t offset_angle_right;
//}offset_angle_t;

//typedef struct
//{
//  offset_angle_t small_island;
//  offset_angle_t big_island;
//}island_offset;

typedef struct
{
  uint8_t init_flag;                    //夹取机构初始化flag(夹取电机校准状态标志位 1已校准 0未校准)
	uint8_t attitude_adjustment_init_flag;//矿石调制机构初始化flag(矿石姿态调制电机校准状态标志位)
	
  uint8_t state;
  uint8_t last_state;

  clamp_flag_t clamp_flag;
  uint8_t island_ctrl;  

  uint8_t clamp_cmd;              //夹取使能位
	uint8_t clamp_throw_away_cmd;   //夹取丢弃使能位
	
	uint8_t store_cmd;              //夹取使能位
	uint8_t store_throw_away_cmd;   //夹取丢弃使能位
	
	uint8_t exchange_cmd;           //兑换使能位
	uint8_t exchange_pick_cmd;   

	uint8_t kb_turn_left_cmd;
	uint8_t kb_turn_right_cmd;
	
	/*控制抬升的状态*/
	uint8_t big_island_upraise_updown_flag      ;
  uint8_t small_island_upraise_updown_flag    ;
	uint8_t ground_upraise_updown_flag          ;
  uint8_t exchange_upraise_updown_flag        ;//控制抬升至兑换点高度
	uint8_t defend_upraise_updown_flag   			  ;
	uint8_t catch_upraise_updown_flag						;
	
	int16_t pit_spd_ref;
  int16_t pit_spd_fdb;
	
	int16_t current[3];
	
  int32_t pit_angle_ref;
  int32_t pit_angle_fdb;
	
  /*初始化角度*/
  int32_t init_angle;
	int32_t clamping_ore_init_angle;
  
  uint32_t  c_clamp_time; //判断遥控   	
	
	uint8_t  identify_cmd; //算法识别完毕，电控运动完毕，可开始夹取动作标志位
  
}clamp_t;

typedef struct
{
	uint8_t init_flag;
	uint8_t state;
	uint8_t last_state;
	
	int16_t spd_ref;
	int16_t spd_fdb;

	int16_t current;
	
	int32_t angle_ref;
	int32_t angle_fdb;
	
	int32_t init_angle;
}store_t;

extern uint8_t clamping_flag;
extern clamp_t    clamp;
extern pick_box_t pick_box;
extern uint8_t exchange_ctrl_gimbal;
extern uint8_t ore_adjust_state;
//extern uint8_t start_exexchange_cmd_state;

extern uint8_t exchange_ctrl_chassis_state;//兑换推落时底盘运动控制

extern float      clamp_pid[6];
extern uint32_t clamp_action_times_old;
extern uint32_t clamp_init_times;	//初始化的延时

extern ramp_t clamp_pit_ramp;

extern clamp_adjust_t clamp_adjust;//夹取机构姿态调整
extern uint8_t ground_clamp_finish;
extern uint8_t clamp_upraise_flag;
extern uint8_t clamp_slide_flag;
extern uint8_t exchange_start_push_flag;
extern uint8_t exchange_view_switch_flag;
extern exchange_sequence_t exchange_sequence;
extern int16_t kb_inhale_servos;
extern int16_t store_servo_angle[2];

void clamp_task(void *parm);

void clamp_param_init(void);
void clamp_init_handler(void);
void clamp_power_on_initialization(void);
void store_handler_old(void);

void small_island_clamp_handler_MKO(void);
void big_island_stright_clamp_handler_MKO(void);
void clamp_ground_handler_MKO(void);
void exchange_handler_MKO(void);	
void big_island_slanted_clamp_handler_MKO(void);

void defend_handler(void);

void clamp_turn_handler(void);
void clamp_turn_2_handler(void);
#endif


