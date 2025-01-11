#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "rc.h"
#include "delay.h"
#include "modeswitch_task.h"
#include "motor_task.h"

#include "User.h"
CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_chassis[4];           //����       3508
/***************ɾ��*******************/
moto_measure_t moto_rescue[2];            //��Ԯ����   2006
moto_measure_t moto_upraise[2];           //̧������   3508
moto_measure_t moto_clamp_pit; 						//��ת��ȡ   3508
moto_measure_t moto_slide[4];							//���			 3508
moto_measure_t moto_manipulator[6];
moto_measure_t moto_store;
/***************ɾ��*******************/

moto_measure_t Motor_CAN1_data[7];
moto_measure_t Motor_CAN2_data[7];

//mpu_data_t mpu_data;

float angle;
float *p_angle = &angle;//p_angleָ�뱣�����angle�ĵ�ַ

float *yaw;
int16_t *gz,*gy;

uint8_t moto_err_id_init_flag=0;
void moto_err_id_init()
{
	moto_manipulator[0].err_id=MANIPULATO_M0_OFFLINE;
	moto_manipulator[1].err_id=MANIPULATO_M1_OFFLINE;
	moto_manipulator[2].err_id=MANIPULATO_M2_OFFLINE;
	moto_manipulator[3].err_id=MANIPULATO_M3_OFFLINE;
	moto_manipulator[4].err_id=MANIPULATO_M4_OFFLINE;
	moto_manipulator[5].err_id=MANIPULATO_M5_OFFLINE;
	moto_err_id_init_flag=1;//��ʼ�����
}
static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(!moto_err_id_init_flag) moto_err_id_init();
	if(_hcan == CAN1)
	{
		switch(message->StdId)//�ϴ��룬���̽�����ʱ����
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
        //������ID��
        i = message->StdId - CAN_3508_M1_ID;
        //���������ݺ꺯��
        moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], message) : encoder_data_handler(&moto_chassis[i], message);
        //��¼ʱ�� -- �쳣����
        err_detector_hook(CHASSIS_M1_OFFLINE + i);
   			}break;
			default:
			{
			}break;
		}
		static uint8_t ID=0;
		ID = message->StdId - 0x201;
		//���������ݺ꺯��
		Motor_CAN1_data[ID].msg_cnt++ <= 50 ? get_moto_offset(&Motor_CAN1_data[ID], message) : encoder_data_handler(&Motor_CAN1_data[ID], message);
//		//��¼ʱ�� -- �쳣���� ����Ҫ��˵�ɣ��쳣����һ�㶼�����������
//    err_detector_hook(CHASSIS_M1_OFFLINE + i);
	}
	else
	{
		static uint8_t ID=0;
		ID = message->StdId - 0x201;
		//���������ݺ꺯��
		Motor_CAN2_data[ID].msg_cnt++ <= 50 ? get_moto_offset(&Motor_CAN2_data[ID], message) : encoder_data_handler(&Motor_CAN2_data[ID], message);
	}
}



void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	
	ptr->speed_rpm     = (int16_t)(message->Data[2] << 8 | message->Data[3]);
  ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);

}


/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}


/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_can1_low_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;//��׼��ʶ��
    TxMessage.IDE = CAN_ID_STD;          // �����ʶ��������Ϊ��׼��ʶ��
    TxMessage.RTR = CAN_RTR_DATA;        //����֡
    TxMessage.DLC = 0x08;                //���ݳ���Ϊ0x08
    TxMessage.Data[0] = iq1 >> 8;       
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;

    CAN_Transmit(CAN1, &TxMessage);
}

void send_can1_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_UPRAISE_ALL_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;	
    CAN_Transmit(CAN1, &TxMessage);	
}

/*���ͼ�ȡ������ʯ̧����̧���������*/
void send_can2_low_cur(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_L_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;	
    CAN_Transmit(CAN2, &TxMessage);	
}

void send_can2_high_cur(int16_t iq1,int16_t iq2,int16_t iq3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_H_ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
	  TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    CAN_Transmit(CAN2, &TxMessage);	
}
//can1�ж�
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
			//����CAN����Ϣ��������������Ϣ���rx1_message
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
			//����������ݣ��������ĸ���������ݼ����ݵľ������弰��С
        STD_CAN_RxCpltCallback(CAN1,&rx1_message);
    }
}

//can2�ж�
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        STD_CAN_RxCpltCallback(CAN2,&rx2_message);
    }
}


