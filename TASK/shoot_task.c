#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"

UBaseType_t shoot_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

shoot_t   shoot;
trigger_t trig;

#if (INFANTRY_NUM == INFANTRY_1)

  /*���ָǿ���*/
  float ccr_open = 500;
  float ccr_close = 1300;
	/* Ħ����ת�� */
	uint16_t normal_speed		= 1150;   	
	
	/* ����ת�� */
	int normal_cshoot				= 3000;		

	/*�������Ʋ���ת��*/
  float heat_limit_pid[3] = {30, 0, 10};
  /*Ħ����pid*/
  float fric_pid[3] = {10, 0, 0};
	
#else
		#error "INFANTRY_NUM define error!"
#endif
	

void shoot_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SHOOT_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SHOOT_SIGNAL)
			{
        if(shoot_mode != SHOOT_DISABLE)
        {
          /*��������*/
          PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, DONE);
          
          /*Ħ����*/
          for(int i=0;i<2;i++)
          {
            PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, DONE); 
          }
          
          shoot_speed_ctrl();						// ����������Ƶ
          ball_storage_ctrl();					// ������Ƶ��ָ�
          fric_wheel_ctrl();						// ����Ħ����
          
          if (shoot.fric_wheel_run)
          {
            shoot_bullet_handler();           
          }
          else
          {
            shoot.shoot_cmd   = 0;
            shoot.c_shoot_cmd = 0;            
            pid_trigger_spd.out = 0;
            trig.angle_ref = moto_trigger.total_angle; //��¼��ǰ���̵������λ
          }
          
        }
        else
        {
          pid_trigger_spd.out = 0;
          shoot.fric_wheel_run = 0;
          turn_off_friction_wheel();  //��ң��֮��Ħ����ͣ��
        }
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
      }
    }
			
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
		
} 



/*������*/
int32_t debug_error = 500/*��⿨����������*/,debug_ref = -5500/*�����ز�����*/;
uint32_t trig_error;
void block_bullet_handler(void)
{
	static uint32_t stall_mark = 0;
	static uint32_t stall_step = 0;
	
  if(((fabs(pid_trigger_spd.set) - fabs(pid_trigger_spd.get)) > 0.8f*fabs(pid_trigger_spd.set)) && stall_step == 0)
    trig_error++;
  else
    trig_error = 0;

	if((stall_step == 0) && (trig_error > debug_error))	//�ж��Ƿ񿨵�
	{	
		stall_step = 1;
		stall_mark = HAL_GetTick();
	}
	if(stall_step == 1)
	{
		trig.spd_ref = debug_ref;
		if(HAL_GetTick() - stall_mark >= 150)//150ms
		{
			stall_step = 0;
			stall_mark = 0;
			trig_error = 0;
		}
	}
}

/*���ģʽѡ��*/
static void shoot_speed_ctrl(void)
{
	/* ����ת�� */
	shoot.fric_wheel_spd			 = normal_speed;
	trig.c_shoot_spd		       = normal_cshoot;
}

/*Ħ���ֿ���*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		turn_on_friction_wheel(shoot.fric_wheel_spd);
	}
	else
	{
		turn_off_friction_wheel();
	}
}

/*��Ħ����*/
static void turn_on_friction_wheel(int16_t speed)
{
//  pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, lspd);
//	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, rspd);
//	glb_cur.fric_cur[0] = pid_fric[0].out;
//	glb_cur.fric_cur[1] = pid_fric[1].out;
  TIM1->CCR1 = speed;
  TIM1->CCR4 = speed;
}

/*�ر�Ħ����*/
static void turn_off_friction_wheel(void)
{
//	pid_calc(&pid_fric[0], moto_fric[0].speed_rpm, 0);
//	pid_calc(&pid_fric[1], moto_fric[1].speed_rpm, 0);
//	glb_cur.fric_cur[0] = pid_fric[0].out;
//	glb_cur.fric_cur[1] = pid_fric[1].out;
  TIM1->CCR1 = 1000;
  TIM1->CCR4 = 1000;
}

/*���ָǿ���*/
static void ball_storage_ctrl(void)
{
  if (shoot.ball_storage_open)
  {
    TIM5->CCR4 = ccr_open;
  }
  else
  {
    TIM5->CCR4 = ccr_close;
  }
}


static void shoot_bullet_handler(void)	
{
  float suplus_heat;
  
  if (shoot.shoot_cmd)//����
  {
    trig.angle_ref = moto_trigger.total_angle + 45;//һ��ת45��
    shoot.shoot_cmd = 0;
  }
  
  pid_calc(&pid_trigger,moto_trigger.total_angle,trig.angle_ref);
                    
  if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.power_heat_data.shooter_heat0 
      > judge_recv_mesg.game_robot_state.shooter_heat0_cooling_limit - 10)
    trig.spd_ref = 0;
  else
    trig.spd_ref = pid_trigger.out;

  if (shoot.c_shoot_cmd)//��������
  {
    if ( !global_err.list[JUDGE_SYS_OFFLINE].err_exist )
		{//����ϵͳ����ģʽ
      suplus_heat = moto_trigger.total_angle + ((judge_recv_mesg.game_robot_state.shooter_heat0_cooling_limit  \
                                                - judge_recv_mesg.power_heat_data.shooter_heat0 - 10) / 10)*45;
      fuzzy_pid_calc(&pid_heat_limit,moto_trigger.total_angle,suplus_heat);
      trig.spd_ref = pid_heat_limit.out;
	  }		
		else 
		{
			trig.spd_ref = trig.c_shoot_spd;
		}
    trig.angle_ref = moto_trigger.total_angle;
  }

	    /* �������� */
  block_bullet_handler();
  
  pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref);
	
	if(trig.spd_ref == 0)
	{
		pid_trigger_spd.out = 0;
	}
}

void shoot_param_init(void)
{
  memset(&shoot, 0, sizeof(shoot_t));
  
  shoot.ctrl_mode      = SHOT_DISABLE;
  
  memset(&trig, 0, sizeof(trigger_t));
  
	shoot.fric_wheel_spd			 = normal_speed;
	trig.c_shoot_spd		       = normal_cshoot;
  
  /*��������*/
  PID_Struct_Init(&pid_heat_limit, heat_limit_pid[0], heat_limit_pid[1], heat_limit_pid[2], 7000, 0, INIT);
  
  /*Ħ����*/
  for(int i=0;i<2;i++)
  {
    PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 8000, 500, INIT); 
  }
  
}


