#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

///*底盘控制*/
//#define RC_DODGE_MODE             ((rc.sw1 == RC_MI) && (rc.sw2 == RC_UP) && (rc.iw >= IW_DN))

///*射击控制*/
//#define RC_SINGLE_SHOOT    				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
//#define RC_CONTINUE_SHOOT  				(rc.sw1 == RC_DN)
//#define RC_CTRL_FRIC_WHEEL 				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
//#define RC_CTRL_BALL_STOR	 				((rc.sw2 == RC_UP) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

/*切换模式操作*/                                                          /*IW_DN 1354 IW_UP 694*/                    
/*当rc.sw1不一样时可切换成救援、补给、障碍块搬运*/

#define RC_IW_DN           ((rc.sw2 == RC_MI) && (glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN)) 
#define RC_IW_UP           ((rc.sw2 == RC_MI) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

#define RC_SW1_DN						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_SW1_UP						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))


/****************删除以下***************************/
/*兑换使能操作*/ 
#define RC_EXCHANGE_ORDINARY_SINGLE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_EXCHANGE_PICK_ORDINARY_SINGLE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))

/*救援控制*/
#define RC_RESCUE_MODE_CMD            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
/*障碍块搬运控制*/
#define RC_BARRIER_CARRY_MODE_UNCMD     ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
#define RC_BARRIER_CARRY_MODE_CMD   ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
/*复活控制*/
#define RC_RESCUE_RESURGENCE_MODE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))

/*21赛季夹取控制*/
/*小资源岛夹取模式选择*/
#define  RC_SMALL_SINGLE_AUTOMATIC_MODE 	        (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_UP)                                   //进入小矿石视觉辅助夹取
#define  RC_SMALL_SINGLE_ORDINARY_MODE 	          (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_DN)                                   //普通夹取模式
#define  RC_SMALL_SINGLE_AUTOMATIC_CLAMP_ONE_MODE (glb_sw.last_last_sw1 == RC_MI && glb_sw.last_sw1 == RC_UP && rc.sw1 == RC_MI)  //小矿石视觉辅助夹取一箱
/*小资源岛普通夹取的使能夹取操作*/
#define  RC_SMALL_ORDINARY_SINGLE_CLAMP_CMD       (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_DN) 
#define  RC_SMALL_ORDINARY_SINGLE_CLAMP_UNCMD     (glb_sw.last_sw1 == RC_DN && rc.sw1 == RC_MI)    //失能操作

/*大资源岛夹取模式选择*/
#define RC_BIG_SINGLE_AUTOMATIC_CLAMP_ONE_MODE    ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))//视觉辅助夹取模式
#define RC_BIG_SINGLE_ORDINARY_CLAMP_MODE         ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI))//手控夹取模式
/*大资源岛普通夹取的使能夹取操作*/
#define  RC_BIG_ORDINARY_SINGLE_CLAMP_CMD         ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //使能操作
#define  RC_BIG_ORDINARY_SINGLE_CLAMP_UNCMD       ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //失能操作
/*存矿*/
#define  RC_ORDINARY_SINGLE_STORE_CMD         ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP)) //使能操作
#define  RC_ORDINARY_SINGLE_STORE_UNCMD       ((glb_sw.last_sw1 == RC_UP) && (rc.sw1 == RC_MI)) //失能操作
#define	 RC_ORDINARY_MIN_STORE_CMD						(rc.sw1 == RC_MI)

/*夹取丢弃*/
#define  RC_CLAMP_THROW_AWAY_CMD      (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_UP) 
#define  RC_CLAMP_THROW_AWAY_UNCMD    (glb_sw.last_sw1 == RC_UP && rc.sw1 == RC_MI) 


/*夹取地上的使能夹取操作*/
#define  RC_GROUND_SINGLE_CLAMP_CMD               ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //使能操作
#define  RC_GROUND_SINGLE_CLAMP_UNCMD             ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //失能操作

/*夹取地上的使能夹取操作*/
#define  RC_CATCH_SINGLE_CLAMP_CMD               ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //使能操作
#define  RC_CATCH_SINGLE_CLAMP_UNCMD             ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //失能操作

/*补给控制*/
#define RC_SUPPLY_CMD1            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))  //补给前动作
#define RC_SUPPLY_CMD2            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))  //补给后动作
#define RC_SUPPLY_START_TIMER     (rc.sw1 != RC_MI)                                  //补给计时操作(补给时间)

/*小猫步*/
#define RC_SMALL_CATWALK       ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP) && (rc.sw2 == RC_UP))

/*抬升初始化*/
#define RC_UPRAISE_INIT           ((rc.ch4 >= 450) && (rc.ch3 >= 450))
/***********************删除以上***************************/

typedef enum
{
	CLAMP_UN_CMD = 0, //
	ACTION_ONE, //确认动作
	ACTION_TWO,//存矿动作
	ACTION_THREE,
}clamp_action_status;

typedef enum
{
	EXCHANGE_UN_CMD  = 0, //
	PICK_ACTION, 
	RELESE_BOX_ACTION,
}exchange_action_status;

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
	IW_UP = 694,		//	min 364		middle 1024	拨轮
	IW_DN = 1354,		//	max 1684
};

typedef enum
{
	CLOCKWISE_ADJUST = 0, //顺时针调整状态
	ONTICLOCKWISE_ADJUST, //逆时针调整状态
}adjusement_t;

typedef struct
{
	/*记录拨杆和拨轮上一次状态*/
  uint8_t last_sw1;
	uint8_t last_last_sw1;//夹取时，因为按键要求过多，因此添加上上次的判断
	
  uint8_t last_sw2;
  uint16_t last_iw;
} sw_record_t;

typedef struct
{
	/*底盘方向*/
  float vx; //前后
  float vy; //左右
  float vw; //转弯
  /*云台方向*/
  float pit_v;
  float yaw_v;
} rc_ctrl_t;


extern rc_ctrl_t rm;
extern sw_record_t glb_sw;
extern adjusement_t adjustment_cmd;
extern clamp_action_status clamp_action;
extern exchange_action_status exchange_action;

extern int32_t adjustment_slide_v;
extern int32_t adjustment_upraise_v;
extern int32_t adjustment_pit_v;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);
void remote_ctrl_gimbal_hook(void);
//void remote_ctrl_shoot_hook(void);

void remote_ctrl_supply_hook(void);
void remote_ctrl_rescue_hook(void);
void remote_ctrl_clamp_hook(void);
void remote_ctrl_barrier_carry_hook(void);
#endif


