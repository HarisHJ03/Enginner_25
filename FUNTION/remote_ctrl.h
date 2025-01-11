#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

///*���̿���*/
//#define RC_DODGE_MODE             ((rc.sw1 == RC_MI) && (rc.sw2 == RC_UP) && (rc.iw >= IW_DN))

///*�������*/
//#define RC_SINGLE_SHOOT    				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
//#define RC_CONTINUE_SHOOT  				(rc.sw1 == RC_DN)
//#define RC_CTRL_FRIC_WHEEL 				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
//#define RC_CTRL_BALL_STOR	 				((rc.sw2 == RC_UP) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

/*�л�ģʽ����*/                                                          /*IW_DN 1354 IW_UP 694*/                    
/*��rc.sw1��һ��ʱ���л��ɾ�Ԯ���������ϰ������*/

#define RC_IW_DN           ((rc.sw2 == RC_MI) && (glb_sw.last_iw < IW_DN) && (rc.iw >= IW_DN)) 
#define RC_IW_UP           ((rc.sw2 == RC_MI) && (glb_sw.last_iw > IW_UP) && (rc.iw <= IW_UP))

#define RC_SW1_DN						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_SW1_UP						((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))


/****************ɾ������***************************/
/*�һ�ʹ�ܲ���*/ 
#define RC_EXCHANGE_ORDINARY_SINGLE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_EXCHANGE_PICK_ORDINARY_SINGLE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))

/*��Ԯ����*/
#define RC_RESCUE_MODE_CMD            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
/*�ϰ�����˿���*/
#define RC_BARRIER_CARRY_MODE_UNCMD     ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
#define RC_BARRIER_CARRY_MODE_CMD   ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
/*�������*/
#define RC_RESCUE_RESURGENCE_MODE_CMD ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))

/*21������ȡ����*/
/*С��Դ����ȡģʽѡ��*/
#define  RC_SMALL_SINGLE_AUTOMATIC_MODE 	        (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_UP)                                   //����С��ʯ�Ӿ�������ȡ
#define  RC_SMALL_SINGLE_ORDINARY_MODE 	          (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_DN)                                   //��ͨ��ȡģʽ
#define  RC_SMALL_SINGLE_AUTOMATIC_CLAMP_ONE_MODE (glb_sw.last_last_sw1 == RC_MI && glb_sw.last_sw1 == RC_UP && rc.sw1 == RC_MI)  //С��ʯ�Ӿ�������ȡһ��
/*С��Դ����ͨ��ȡ��ʹ�ܼ�ȡ����*/
#define  RC_SMALL_ORDINARY_SINGLE_CLAMP_CMD       (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_DN) 
#define  RC_SMALL_ORDINARY_SINGLE_CLAMP_UNCMD     (glb_sw.last_sw1 == RC_DN && rc.sw1 == RC_MI)    //ʧ�ܲ���

/*����Դ����ȡģʽѡ��*/
#define RC_BIG_SINGLE_AUTOMATIC_CLAMP_ONE_MODE    ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))//�Ӿ�������ȡģʽ
#define RC_BIG_SINGLE_ORDINARY_CLAMP_MODE         ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI))//�ֿؼ�ȡģʽ
/*����Դ����ͨ��ȡ��ʹ�ܼ�ȡ����*/
#define  RC_BIG_ORDINARY_SINGLE_CLAMP_CMD         ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //ʹ�ܲ���
#define  RC_BIG_ORDINARY_SINGLE_CLAMP_UNCMD       ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //ʧ�ܲ���
/*���*/
#define  RC_ORDINARY_SINGLE_STORE_CMD         ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP)) //ʹ�ܲ���
#define  RC_ORDINARY_SINGLE_STORE_UNCMD       ((glb_sw.last_sw1 == RC_UP) && (rc.sw1 == RC_MI)) //ʧ�ܲ���
#define	 RC_ORDINARY_MIN_STORE_CMD						(rc.sw1 == RC_MI)

/*��ȡ����*/
#define  RC_CLAMP_THROW_AWAY_CMD      (glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_UP) 
#define  RC_CLAMP_THROW_AWAY_UNCMD    (glb_sw.last_sw1 == RC_UP && rc.sw1 == RC_MI) 


/*��ȡ���ϵ�ʹ�ܼ�ȡ����*/
#define  RC_GROUND_SINGLE_CLAMP_CMD               ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //ʹ�ܲ���
#define  RC_GROUND_SINGLE_CLAMP_UNCMD             ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //ʧ�ܲ���

/*��ȡ���ϵ�ʹ�ܼ�ȡ����*/
#define  RC_CATCH_SINGLE_CLAMP_CMD               ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) //ʹ�ܲ���
#define  RC_CATCH_SINGLE_CLAMP_UNCMD             ((glb_sw.last_sw1 == RC_DN) && (rc.sw1 == RC_MI)) //ʧ�ܲ���

/*��������*/
#define RC_SUPPLY_CMD1            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))  //����ǰ����
#define RC_SUPPLY_CMD2            ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))  //��������
#define RC_SUPPLY_START_TIMER     (rc.sw1 != RC_MI)                                  //������ʱ����(����ʱ��)

/*Сè��*/
#define RC_SMALL_CATWALK       ((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP) && (rc.sw2 == RC_UP))

/*̧����ʼ��*/
#define RC_UPRAISE_INIT           ((rc.ch4 >= 450) && (rc.ch3 >= 450))
/***********************ɾ������***************************/

typedef enum
{
	CLAMP_UN_CMD = 0, //
	ACTION_ONE, //ȷ�϶���
	ACTION_TWO,//�����
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
	IW_UP = 694,		//	min 364		middle 1024	����
	IW_DN = 1354,		//	max 1684
};

typedef enum
{
	CLOCKWISE_ADJUST = 0, //˳ʱ�����״̬
	ONTICLOCKWISE_ADJUST, //��ʱ�����״̬
}adjusement_t;

typedef struct
{
	/*��¼���˺Ͳ�����һ��״̬*/
  uint8_t last_sw1;
	uint8_t last_last_sw1;//��ȡʱ����Ϊ����Ҫ����࣬���������ϴε��ж�
	
  uint8_t last_sw2;
  uint16_t last_iw;
} sw_record_t;

typedef struct
{
	/*���̷���*/
  float vx; //ǰ��
  float vy; //����
  float vw; //ת��
  /*��̨����*/
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


