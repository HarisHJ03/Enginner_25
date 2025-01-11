#include "judge_tx_data.h"
#include "dma.h"
#include "judge_rx_data.h"
#include "string.h"
#include "shoot_task.h"
#include "remote_ctrl.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "barrier_carry_task.h"
#include "math.h"
#include "clamp_task.h"
#include "upraise_task.h"
#include "rescue_task.h"

#include "clamp.h"

judge_txdata_t judge_send_mesg;              //����ϵͳ���ͽṹ�壬�û��Զ��巢�͵����ݴ�������棬��ʱ�������debug��鿴���޸�
static SemaphoreHandle_t judge_txdata_mutex;
fifo_s_t  judge_txdata_fifo;                 //������ʱ������ݵ�����

static uint8_t   judge_txdata_buf[JUDGE_TX_FIFO_BUFLEN];

void judgement_tx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  judge_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_TX_FIFO_BUFLEN, judge_txdata_mutex);
}

//����������ͨ��
void judgement_client_packet_pack(uint8_t *p_data)
{
	uint8_t current_robot_id = 0;
	uint16_t receiver_ID = 0;
	
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //��ȡ��ǰ�����˵�id
	
	//�˴�ѡ���receiver_ID��������ѡ��
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_SENTRY_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_SENTRY_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;

	}

	judge_send_mesg.ext_student_interactive_data.data_cmd_id = RobotCommunication;
	judge_send_mesg.ext_student_interactive_data.sender_ID = (uint16_t)current_robot_id;
	judge_send_mesg.ext_student_interactive_data.receiver_ID = receiver_ID;
	
	//���Զ�������ݸ��Ƶ����ͽṹ����
	memcpy(&judge_send_mesg.ext_student_interactive_data.data[0], p_data,sizeof(judge_send_mesg.ext_student_interactive_data.data));
	//�ú����Ĺ���Ϊ����Ҫ���͵����ݴ����������һ��ͨ������3���͸�����ϵͳ
	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.ext_student_interactive_data,
									 STUDENT_DATA_LENGTH, DN_REG_ID);
}

/*�������ݾ�����Ҫ��ʵ����ʾЧ���޸ĵģ������ڻ�ͼʱ������������ֵ���ṹ�壬����������debug�иı�ṹ���
	ֵ���Ⱥ��ʵ�ֵȷ���������ٽ����ȷ����ֵ�Գ�����ֵ���ṹ��*/
uint32_t debug_start_angle = 0;
uint32_t debug_end_angle   = 0;
uint32_t debug_width       = 0;
uint32_t debug_start_x     = 800;
uint32_t debug_start_y     = 0;
uint32_t debug_radius      = 0;
uint32_t debug_end_x       = 1400;
uint32_t debug_end_y       = 0;

/*���������������Ҫ���͵��ַ������Ϊ30  �������*/
char text1[30] = {"Q-PIT  E-ORE"};
char text2[30] = {"A-RES-L S-RES-R D-UP"};
char text3[30] = {"V-view switch R-RESET"};
char text4[30] = {"Shift-Ctrl-R ALL-RESET"};

/*����*/
char Pick_box[30] = {"pick box"};
char Height_num[30] = {"H:0000"};
/*����*/
char Pick_box0[30] = {"ZERO"};
char Pick_box1[30] = {"ONE"};
char Pick_box2[30] = {"TWO"};

/*����ģʽ*/
char chassis1[30] = {"normal"};
char chassis2[30] = {"barrier_carry"};
char chassis3[30] = {"exchange"};
char chassis4[30] = {"rescue"};
char chassis5[30] = {"small island"};
char chassis6[30] = {"catch"};
char chassis7[30] = {"big island"};
char chassis8[30] = {"ground "};

char on[30] = {"O"};
char off[30] = {"F"};

char exchange_up[30] = {"UP"};
char exchange_mi[30] = {"MI"};
char exchange_down[30] ={"DN"};


//�����ַ�
static void client_graphic_draw_text1 (void)
{
  memcpy(&judge_send_mesg.ext_client_custom_character_text1.data[0],text1,sizeof(text1));
	
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.graphic_name[0] = Text1;         //ͼ�����֣��Զ��壩
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.operate_tpye    = Add;           //��Ӳ���
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.graphic_tpye    = Character;     //�ַ�
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.layer           = layer0;        //ͼ��
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.color           =  Orange ;     //��ɫ
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.start_angle     =  20 ;          //�����С
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.end_angle       = sizeof(text1); //�ַ�����
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.width           = 2;             //�������
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.start_x         = 1440;          //���x����     
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.start_y         = 890;
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_text1.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_text2 (void)
{
	
	memcpy(&judge_send_mesg.ext_client_custom_character_text2.data[0],text2,sizeof(text2));
	
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.graphic_name[0] = Text2;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.layer           = layer0;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.color           =  Orange ;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.end_angle       = sizeof(text2);
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.start_x         = 1440;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.start_y         = 840;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_text2.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_text3 (void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_text3.data[0],text3,sizeof(text3));
	
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.graphic_name[0] = Text3;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.layer           = layer0;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.color           =  Orange ;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.end_angle       = sizeof(text3);
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.start_x         = 1440;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.start_y         = 790;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_text3.grapic_data_struct.end_y           = 0;
}

/********************************��ʯ����*************************************************/
static void client_graphic_draw_pick(void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_pick.data[0],Pick_box,sizeof(Pick_box));
	
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.graphic_name[0] = Pick;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.layer           = layer1;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.end_angle       = sizeof(Pick_box);
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.start_x         = 50;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.start_y         = 820;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_pick.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_pick_box_init(void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_pick_box.data[0],Pick_box0,sizeof(Pick_box0));
	
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_name[0] = Pick_Box;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.layer           = layer1;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_angle       = sizeof(Pick_box0);
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_x         = 250;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_y         = 820;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_pick_box0(void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_pick_box.data[0],Pick_box0,sizeof(Pick_box0));
	
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_name[0] = Pick_Box;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.layer           = layer1;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_angle       = sizeof(Pick_box0);
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_x         = 250;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_y         = 820;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_pick_box1(void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_pick_box.data[0],Pick_box1,sizeof(Pick_box1));
	
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_name[0] = Pick_Box;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.layer           = layer1;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.color           = Yellow ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_angle       = sizeof(Pick_box1);
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_x         = 250;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_y         = 820;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_pick_box2(void)
{
	memcpy(&judge_send_mesg.ext_client_custom_character_pick_box.data[0],Pick_box2,sizeof(Pick_box2));
	
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_name[0] = Pick_Box;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.layer           = layer1;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.color           = Orange;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_angle       = sizeof(Pick_box2);
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_x         = 250;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.start_y         = 820;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_pick_box.grapic_data_struct.end_y           = 0;
}

/********************************CHASSIS*******************************************************/
static void client_graphic_draw_chassis1(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis1, sizeof(chassis1));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis1) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_chassis2(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis2, sizeof(chassis2));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis2) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_chassis3(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis3, sizeof(chassis3));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis3) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}	
static void client_graphic_draw_chassis4(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis4, sizeof(chassis4));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis4) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}	
static void client_graphic_draw_chassis5(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis5, sizeof(chassis5));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis5) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}	
static void client_graphic_draw_chassis6(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis6, sizeof(chassis6));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis6) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_chassis7(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis7, sizeof(chassis7));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           =  Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis7) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_chassis8(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis8, sizeof(chassis8));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           =  Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     =  20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis8) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 60;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_chassis_init(void) //normal_init
{
	memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], chassis1, sizeof(chassis1));
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = Chassis;
		
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer           = layer3;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle       = sizeof(chassis1) ;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x         = 180;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y           = 0;
}	

static void client_graphic_draw_OFF_init(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_mode_state.data[0], off, sizeof(off));
		
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_name[0] = Mode_State;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.layer           = layer4;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_angle       = sizeof(off) ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_x         = 280;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_OFF(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_mode_state.data[0], off, sizeof(off));
		
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_name[0] = Mode_State;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.layer           = layer4;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.color           = Green ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_angle       = sizeof(off) ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_x         = 280;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_y           = 0;
}
static void client_graphic_draw_ON(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_mode_state.data[0], on, sizeof(on));
		
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_name[0] = Mode_State;
		
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.layer           = layer4;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.color           = Orange ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_angle       = sizeof(on) ;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_x         = 280;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.start_y         = 870;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_mode_state.grapic_data_struct.end_y           = 0;
}

static void client_graphic_draw_Height(void) 
{
	memcpy(&judge_send_mesg.ext_client_custom_character_mode_state.data[0], Height_num, sizeof(Height_num));
		
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.graphic_name[0] = HEIGHT;
		
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.operate_tpye    = Add;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.layer           = layer5;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.color           = Amaranth ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_angle       = sizeof(Height_num) ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_x         = 600;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_y         = 600;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_y           = 0;
}

static void client_graphic_draw_Height_change(void) 
{
		Height_num[2] = (uint32_t)(upraise.angle_fdb[0] - upraise.init_angle[0])/1000 + 48;
		Height_num[3] = (uint32_t)((upraise.angle_fdb[0] - upraise.init_angle[0])/100)%10 +48;
		Height_num[4] = (uint32_t)(((upraise.angle_fdb[0] - upraise.init_angle[0])/10)%100)%10 +48;
		Height_num[5] = (uint32_t)(upraise.angle_fdb[0] - upraise.init_angle[0])%10 +48;
	
	memcpy(&judge_send_mesg.ext_client_custom_character_upraise.data[0], Height_num, sizeof(Height_num));
		
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.graphic_name[0] = HEIGHT;
		
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.operate_tpye    = Change;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.graphic_tpye    = Character;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.layer           = layer5;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.color           = Amaranth ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_angle     = 20 ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_angle       = sizeof(Height_num) ;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.width           = 2;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_x         = 600;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.start_y         = 600;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.radius          = 0;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_x           = 0;
	judge_send_mesg.ext_client_custom_character_upraise.grapic_data_struct.end_y           = 0;
}
/***************************************ͼ��*********************************************************/

//����x:(960��1202.75),y:(540��280)
//static void client_graphic_draw_catch_line(void)//�ս��� 
//{
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].graphic_name[0] = line_1c;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].operate_tpye    = Add;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].graphic_tpye    = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].layer           = layer2;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].color           = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].start_angle     = 0;	
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].end_angle       = 0;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].width           = 3;             //�������
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].start_x         = 1060;           //���
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].start_y         = 820;             //���
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].radius          = 0;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].end_x           = 1060;           //�յ�
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[0].end_y           = 190;            //�յ�
//	
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].graphic_name[0] = line_2c;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].operate_tpye    = Add;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].graphic_tpye    = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].layer           = layer2;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].color           = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].start_angle     = 0;	
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].end_angle       = 0;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].width           = 3;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].start_x         = 860;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].start_y         = 820;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].radius          = 0;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].end_x           = 860;
//	judge_send_mesg.ext_client_custom_graphic_catch_line.grapic_data_struct[1].end_y           = 190;
//}
//ɾ��ͼ��
static void client_graphic_delete_pick (void)
{
	judge_send_mesg.ext_client_custom_graphic_delete.operate_tpye = Delete_graph;
	judge_send_mesg.ext_client_custom_graphic_delete.layer        = layer1;
}
/*�û��Զ����־λ��Ŀǰ���뷨�ǿ���ͨ���������л�ͼ˳��֮����л�*/

//ͼ�α�־λ���˴���־λ���������ƴӿ��ֵ�����������Ҫɾ����ͼ�Σ�
uint8_t Line_mask    	  = 0; //����
uint8_t text_mask       = 1;
uint8_t Chassis_mask 	  = 1;//ѡ����ʾ��ǰģʽ
uint8_t mode_state_mask = 1;
uint8_t upraise_mask		= 1;

uint8_t text_twist    = 1;

//�ͻ����Զ���UI����
void judgement_client_graphics_draw_pack(text_twist_t text_twist)
{
	uint8_t current_robot_id = 0;
	uint16_t receiver_ID = 0;
	static uint8_t i;
	static uint8_t again_flag;
	
	/*���·��Ͳ���  �������ȼ��ϸ�*/
	if(rc.sw2 != RC_DN && !again_flag)
	{
		/*��������ʱ ��ɾ��*/
		judge_send_mesg.ext_client_custom_graphic_delete.data_cmd_id  = Client_Draw_Five_Graph;
		judge_send_mesg.ext_client_custom_graphic_delete.sender_ID    = (uint16_t)current_robot_id;
		judge_send_mesg.ext_client_custom_graphic_delete.receiver_ID  = receiver_ID;
		
		client_graphic_delete_pick();
		
		data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_delete,
									sizeof(judge_send_mesg.ext_client_custom_graphic_delete), DN_REG_ID);
		
		
		Line_mask  = 1;
		text_mask  = 1;
		
		again_flag = 1;
	}
	else if(rc.sw2 == RC_UP && glb_sw.last_sw1 == RC_MI && rc.sw1 == RC_DN)
	{
		again_flag = 0;
	}
	/*��ȡ��ǰ�����˵�id*/
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id; 
	
	//�Զ���UI receiver_IDֻ��ѡ��ǰ�����˵Ķ�Ӧ�Ŀͻ���
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = RED_HERO_CLIENT_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = RED_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = RED_AERIAL_CLIENT_ID;
		}break;

		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = RED_INFANTRY3_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = RED_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = RED_INFANTRY5_CLIENT_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = BLUE_HERO_CLIENT_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = BLUE_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = BLUE_AERIAL_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = BLUE_INFANTRY3_CLIENT_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = BLUE_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = BLUE_INFANTRY5_CLIENT_ID;
		}break;

	}
	/*���Ƴ���  �������*/
	if(Line_mask == 1)
	{
		/*һֱ����5�γ�������� ��ֹ���մ���*/
		if(i++ < 5)
		{
		}
		else
		{	
			Line_mask = 2;
			
			i = 0;
		}
	}
/*��ʾ����Ҫ�仯���ַ�*/
	if(Line_mask == 2)
	{
			if(text_mask == 1)
			{
				if(i++ <5)
				{
					judge_send_mesg.ext_client_custom_character_text1.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_text1.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_text1.receiver_ID = receiver_ID;
					
					client_graphic_draw_text1();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_text1,
													sizeof(judge_send_mesg.ext_client_custom_character_text1), DN_REG_ID);
				}
				else
				{
				text_mask =2;
					i = 0;
				}
			}
			else if(text_mask == 2)
			{
				if(i++ < 5)
				{
					judge_send_mesg.ext_client_custom_character_text2.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_text2.sender_ID = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_text2.receiver_ID = receiver_ID;

					client_graphic_draw_text2();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_text2,
													sizeof(judge_send_mesg.ext_client_custom_character_text2), DN_REG_ID);
				}
				else
				{
					text_mask =3;
					i = 0;
				}
			}
			else if(text_mask == 3)
			{
				if(i++ <5)
				{
					judge_send_mesg.ext_client_custom_character_text3.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_text3.sender_ID = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_text3.receiver_ID = receiver_ID;
					
					client_graphic_draw_text3();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_text3,
													sizeof(judge_send_mesg.ext_client_custom_character_text3), DN_REG_ID);
				}
				else
				{
					text_mask =4;
					i = 0;
				}
			}
			
			/*��һ��ʱ ���͵���ģʽΪ��ͨģʽ*/
			else if(text_mask == 4)
			{		
				if(i++ <5)
				{
					judge_send_mesg.ext_client_custom_character_chassis .data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID    = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID  = receiver_ID;
					/*����ģʽ*/
					client_graphic_draw_chassis_init();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				}
				else
				{
					text_mask    = 7;
					
					Chassis_mask = 2;
					
					i = 0;
				}
			}		
			/*��һ��ʱ ��ȡ����Ĭ����ʾΪ����*/
			else if(text_mask == 7)
			{		
				if(i++ < 5)
				{
					judge_send_mesg.ext_client_custom_character_pick.data_cmd_id  = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_pick.sender_ID    = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_pick.receiver_ID  = receiver_ID;
					
					/*��ʯ��ʾ*/
					client_graphic_draw_pick();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick,
													sizeof(judge_send_mesg.ext_client_custom_character_pick), DN_REG_ID);
				}
				else if(i++ < 10)
				{
					judge_send_mesg.ext_client_custom_character_pick_box.data_cmd_id  = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_pick_box.sender_ID    = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_pick_box.receiver_ID  = receiver_ID;
					
					/*��ʯ��ʾ*/
					client_graphic_draw_pick_box_init();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick_box,
													sizeof(judge_send_mesg.ext_client_custom_character_pick_box), DN_REG_ID);
				}
				else
				{
					text_mask    = 8;
					i = 0;
				}
			}
			else if(text_mask == 8)
			{
				if(i++ < 5)
				{
					judge_send_mesg.ext_client_custom_character_mode_state.data_cmd_id  = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_mode_state.sender_ID    = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_mode_state.receiver_ID  = receiver_ID;
					
					/*ģʽ״̬��ʾ*/
					client_graphic_draw_OFF_init();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_mode_state,
													sizeof(judge_send_mesg.ext_client_custom_character_mode_state), DN_REG_ID);				
				}
				else
				{
					text_mask = 9;
					i = 0;
				}
			}
			else if(text_mask == 9)
			{
				if(i++ < 5)
				{
					judge_send_mesg.ext_client_custom_character_upraise.data_cmd_id  = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_upraise.sender_ID    = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_upraise.receiver_ID  = receiver_ID;
					
					/*��ʯ��ʾ*/
					client_graphic_draw_Height();
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_upraise,
													sizeof(judge_send_mesg.ext_client_custom_character_upraise), DN_REG_ID);				
				}
				else
				{
					text_mask = 10;
					Line_mask = 3;
					i = 0;
				}				
			}
		}
	/*ʵʱ��ʾ���̵�ǰģʽ*/
	if(text_twist == CHASSIS_MODE)
	{
		/*����ģʽ�л�  ���������Ѿ���ȫ��������ٿ�ʼʵʱ��ʾ���̵�ģʽ*/
		if(Chassis_mask == 2)
		{
			switch (chassis_mode)
			{
				case CHASSIS_BARRIER_CARRY_MODE:
				{
					Chassis_mask = 3;
				}break;
				case CHASSIS_EXCHANGE_MODE:
				{
					Chassis_mask = 4;
				}break;	
				case CHASSIS_RESCUE_MODE:
				{
					Chassis_mask = 5;
				}break;
				case CHASSIS_CLAMP_SMALL_ISLAND_MODE:
				{
					Chassis_mask = 6;
				}break;
				case CHASSIS_SUPPLY_MODE:
				{
					Chassis_mask = 7;
				}break;
				case CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE:
				{
					Chassis_mask = 8;
				}break;		
				
				case CHASSIS_GROUND_MODE:
				{
					Chassis_mask = 9;
				}break;
				
				default:
				{
					Chassis_mask = 10;//��ͨģʽ
				}break;
			}	
		}
			
			if(Chassis_mask == 3 && chassis_mode == CHASSIS_BARRIER_CARRY_MODE)
			{
				judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
				client_graphic_draw_chassis2();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
										sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
				Chassis_mask = 2;//���ƽ����жϵ���ģʽ
			}
			else if(Chassis_mask == 4 && chassis_mode == CHASSIS_EXCHANGE_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis3();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
				
			}
			else if(Chassis_mask == 5 && chassis_mode == CHASSIS_RESCUE_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis4();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
			else if(Chassis_mask == 6 && chassis_mode == CHASSIS_CLAMP_SMALL_ISLAND_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis5();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
			else if(Chassis_mask == 7 && chassis_mode == CHASSIS_SUPPLY_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis6();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
			else if(Chassis_mask == 8 && chassis_mode == CHASSIS_CLAMP_BIG_ISLAND_STRAIGHT_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis7();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
			else if(Chassis_mask == 9 && chassis_mode == CHASSIS_GROUND_MODE)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis8();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
			else if(Chassis_mask == 10)
			{
					judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
					judge_send_mesg.ext_client_custom_character_chassis.sender_ID   = (uint16_t)current_robot_id;
					judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				
					client_graphic_draw_chassis1();
				
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
													sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
				
					Chassis_mask = 2;
			}
	}
	/*ʵʱ��ʾ��ȡ����*/
	if(text_twist == PICK_BOX)
	{
		switch(have_box_number)
		{
			case 0:
			{			
				judge_send_mesg.ext_client_custom_character_pick_box.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_pick_box.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_pick_box.receiver_ID  = receiver_ID;

				client_graphic_draw_pick_box0();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick_box,
												sizeof(judge_send_mesg.ext_client_custom_character_pick_box), DN_REG_ID);
			}break;
			case 1:
			{
				judge_send_mesg.ext_client_custom_character_pick_box.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_pick_box.sender_ID   = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_pick_box.receiver_ID = receiver_ID;
				
				client_graphic_draw_pick_box1();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick_box,
									sizeof(judge_send_mesg.ext_client_custom_character_pick_box), DN_REG_ID);
			}break;
			case 2:
			{
				judge_send_mesg.ext_client_custom_character_pick_box.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_pick_box.sender_ID   = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_pick_box.receiver_ID = receiver_ID;
				
				client_graphic_draw_pick_box2();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick_box,
										sizeof(judge_send_mesg.ext_client_custom_character_pick_box), DN_REG_ID);
			}break;
			default:
			{
				judge_send_mesg.ext_client_custom_character_pick_box.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_pick_box.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_pick_box.receiver_ID  = receiver_ID;

				client_graphic_draw_pick_box0();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_pick_box,
												sizeof(judge_send_mesg.ext_client_custom_character_pick_box), DN_REG_ID);
			}break;
		}

	}
	
	if(text_twist == MODE_STATE)
	{
//		if(chassis_mode != CHASSIS_BARRIER_CARRY_MODE && chassis_mode != CHASSIS_EXCHANGE_MODE)
//		{
			if(clamp.clamp_flag == CLAMPING)
			{
				judge_send_mesg.ext_client_custom_character_mode_state.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_mode_state.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_mode_state.receiver_ID  = receiver_ID;

				client_graphic_draw_ON();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_mode_state,
												sizeof(judge_send_mesg.ext_client_custom_character_mode_state), DN_REG_ID);				
			}
			else
			{
				judge_send_mesg.ext_client_custom_character_mode_state.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_mode_state.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_mode_state.receiver_ID  = receiver_ID;

				client_graphic_draw_OFF();
				
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_mode_state,
												sizeof(judge_send_mesg.ext_client_custom_character_mode_state), DN_REG_ID);					
			}
	}
	
	if(text_twist == UPRAISE_HEIGHT)
	{
				judge_send_mesg.ext_client_custom_character_upraise.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_upraise.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_upraise.receiver_ID  = receiver_ID;

			client_graphic_draw_Height_change();

				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_upraise,
												sizeof(judge_send_mesg.ext_client_custom_character_upraise), DN_REG_ID);		
	}
	if(text_twist == AUXILIARY_LINE)
	{
				judge_send_mesg.ext_client_custom_graphic_catch_line.data_cmd_id  = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_graphic_catch_line.sender_ID    = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_graphic_catch_line.receiver_ID  = receiver_ID;
				
//		if(chassis_mode==CATCH_MODE)
//				client_graphic_draw_catch_line();

				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_catch_line,
												sizeof(judge_send_mesg.ext_client_custom_graphic_catch_line), DN_REG_ID);		
	}
}

