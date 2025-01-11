/*****************************************************
	  由凌浩天写于2024/4/-
		
		过去，增加一个电机，要去bsp_can文件调整接收格式，
去common改角度转换比与发送格式，去相应功能文件增加它的初始化
		过去，机械臂与抬升和伸出的代码逻辑并不一致，改起来劳心劳力
	  
		现统一manipulator、slide、chassis任务控制逻辑，
并将电机的电调ID、速度与角度环选择等全放在头文件统一修改
剩下交给函数自行处理

		为统一配置的时代欢呼吧！
*****************************************************/
#include "motor_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "clamp.h"
#include "motor_8010.h"
#include "stm32f4xx.h"
int16_t test_angle1=2500;
int16_t small_test1=100;
UBaseType_t Motor_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t comm_8010_task_Handle;
extern int16_t go_8010_current;

extern float  send_current_8010_1;
extern float  send_current_8010_2;
extern float send_ref_angle_8010_1;
extern float send_ref_angle_8010_2;
extern float send_fdb_angle_8010_1;
extern float send_fdb_angle_8010_2;


motor_t Motor[MOTOR_NUMBER];

uint8_t Motor_init_state=INIT_NEVER;
uint8_t All_Offset_Angle_init_state=0;

uint32_t Servo_Test;
float test_pid_speed_out=0;
float test_pid=0;
int32_t test_angle;
float test_ref[14];
float l_test_ref[14];

//这个初始化电机
void Motor_base_init()
{
	motor_t Motor_Struct;
	//复制粘贴后只用改一个名字就能配置相应电机，比起以前强爆啦！
	{// CHASSIS INIT
    Motor_Struct.MOTOR_NAME = CHASSIS_FR;
    Motor_Struct.MOTOR_TYPE = M3508;
    Motor_Struct.Brushless.CAN_ID = 1;
    Motor_Struct.Brushless.ESC_ID = 1;
    Motor_Struct.Brushless.speed = 0; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 0;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// UPRISE INIT
    Motor_Struct.MOTOR_NAME = UPRISE_L;
    Motor_Struct.MOTOR_TYPE = M3508;
    Motor_Struct.Brushless.CAN_ID = 1;
    Motor_Struct.Brushless.ESC_ID = 5;
		Motor_Struct.Brushless.Angle_to_Speed_mode=0;//开切换
    Motor_Struct.Brushless.speed = 1500; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -700;
		

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// CLAMP_YAW INIT
    Motor_Struct.MOTOR_NAME = CLAMP_YAW;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM2;
    Motor_Struct.Servo.Compare = 3; // PA2
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// CLAMP_PITCH INIT
    Motor_Struct.MOTOR_NAME = CLAMP_PITCH;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM4;
    Motor_Struct.Servo.Compare = 3; // PD14
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// CLAMP_ROLL INIT
    Motor_Struct.MOTOR_NAME = CLAMP_ROLL;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM4;
    Motor_Struct.Servo.Compare = 2; // PD13
    Motor_Struct.Servo.Rotation_range = 0;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// SLIDE INIT
    Motor_Struct.MOTOR_NAME = SLIDE_L;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 1;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 1000; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -1500;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// SLIDE_CLAMP INIT
    Motor_Struct.MOTOR_NAME = joint_3_roll;
    Motor_Struct.MOTOR_TYPE = M2006;
    Motor_Struct.Brushless.CAN_ID = 2;
    Motor_Struct.Brushless.ESC_ID = 3;
		Motor_Struct.Brushless.Speed_or_Angle_flag=1;
    Motor_Struct.Brushless.speed = 0; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 0;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// EXCHANGE_YAW INIT
    Motor_Struct.MOTOR_NAME = joint_1;
    Motor_Struct.MOTOR_TYPE = go_8010;
		Motor_Struct.Brushless.GO_ID=GO_8010_1;
    Motor_Struct.Brushless.speed = 0; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = 120;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// EXCHANGE_PITCH INIT
    Motor_Struct.MOTOR_NAME = joint_2;
    Motor_Struct.MOTOR_TYPE = go_8010; 
		Motor_Struct.Brushless.GO_ID=3; 
    Motor_Struct.Brushless.speed = 0; // 大角度差下速度
    Motor_Struct.Brushless.state = 0;
    Motor_Struct.Brushless.offset_angle_init_flag = 0;
		Motor_Struct.Brushless.offset_angle_init_speed = -1000;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}

	{// EXCHANGE_ROLL INIT
    Motor_Struct.MOTOR_NAME = EXCHANGE_ROLL;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM4;
    Motor_Struct.Servo.Compare = 4; // PD15
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	
	{// GIMBAL_YAW
    Motor_Struct.MOTOR_NAME = GIMBAL_YAW;
    Motor_Struct.MOTOR_TYPE = SERVO;
    Motor_Struct.Servo.TIM = TIM2;
    Motor_Struct.Servo.Compare = 1; // PA0
    Motor_Struct.Servo.Rotation_range = 270;

    memcpy(&Motor[Motor_Struct.MOTOR_NAME], &Motor_Struct, sizeof(Motor_Struct));
    memset(&Motor_Struct, 0, sizeof(Motor_Struct));
	}
	
	Motor_pid_init(DONE);
	Motor_angle_init();
	
	//Motor_base_init_copy(CHASSIS_FR,CHASSIS_BR);//配置复制
	//Motor_base_init_copy(UPRISE_L,UPRISE_R);//配置复制
	Motor_base_init_copy(SLIDE_L,SLIDE_R);//配置复制
	
	//Motor_base_init_reversal(SLIDE_R);//翻转SLIDE_R配置
//	Motor[SLIDE_R].MOTOR_TURN_FLAG=M_TURN;
}

//初始化pid
void Motor_pid_init(INIT_STATUS init_status) 
{
	pid_motor_parameter_t PID_Motor_parameter_Struct;

	// CHASSIS_FR
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CHASSIS_FR;
    PID_Motor_parameter_Struct.angle.p = 60.0;
    PID_Motor_parameter_Struct.angle.i = 0.01;
    PID_Motor_parameter_Struct.angle.d = 0.1;
    PID_Motor_parameter_Struct.angle.max_out = 30000;
    PID_Motor_parameter_Struct.angle.integral_limit = 30000;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 30000;
    PID_Motor_parameter_Struct.speed.integral_limit = 30000;

    Motor_PID_Struct_Init(&Motor[CHASSIS_FR], PID_Motor_parameter_Struct, init_status);
	}

	// UPRISE_L
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = UPRISE_L;
    PID_Motor_parameter_Struct.angle.p = 30;
    PID_Motor_parameter_Struct.angle.i = 0.2;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 6000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.001;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 8000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[UPRISE_L], PID_Motor_parameter_Struct, init_status);
	}

	// CLAMP_YAW
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CLAMP_YAW;
    PID_Motor_parameter_Struct.angle.p = 0;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 0;
    PID_Motor_parameter_Struct.angle.integral_limit = 0;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 1000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1000;

    Motor_PID_Struct_Init(&Motor[CLAMP_YAW], PID_Motor_parameter_Struct, init_status);
	}

	// CLAMP_PITCH
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CLAMP_PITCH;
    PID_Motor_parameter_Struct.angle.p = 0;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 0;
    PID_Motor_parameter_Struct.angle.integral_limit = 0;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 30000;
    PID_Motor_parameter_Struct.speed.integral_limit = 30000;

    Motor_PID_Struct_Init(&Motor[CLAMP_PITCH], PID_Motor_parameter_Struct, init_status);
	}

	// CLAMP_ROLL
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = CLAMP_ROLL;
    PID_Motor_parameter_Struct.angle.p = 0;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 0;
    PID_Motor_parameter_Struct.angle.integral_limit = 0;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 30000;
    PID_Motor_parameter_Struct.speed.integral_limit = 30000;

    Motor_PID_Struct_Init(&Motor[CLAMP_ROLL], PID_Motor_parameter_Struct, init_status);
	}

	// SLIDE_L
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = SLIDE_L;
    PID_Motor_parameter_Struct.angle.p = 20;
    PID_Motor_parameter_Struct.angle.i = 0.1;
    PID_Motor_parameter_Struct.angle.d = 0.02;
    PID_Motor_parameter_Struct.angle.max_out = 1000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.02;
    PID_Motor_parameter_Struct.speed.max_out = 4000;
    PID_Motor_parameter_Struct.speed.integral_limit = 500;

    Motor_PID_Struct_Init(&Motor[SLIDE_L], PID_Motor_parameter_Struct, init_status);
	}
	// SLIDE_CLMP
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_3_roll;
    PID_Motor_parameter_Struct.angle.p = 90;
    PID_Motor_parameter_Struct.angle.i = 0.0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 7000;
    PID_Motor_parameter_Struct.angle.integral_limit = 500;

    PID_Motor_parameter_Struct.speed.p = 20.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.;
    PID_Motor_parameter_Struct.speed.max_out = 2000;
    PID_Motor_parameter_Struct.speed.integral_limit = 1500;

    Motor_PID_Struct_Init(&Motor[joint_3_roll], PID_Motor_parameter_Struct, init_status);
	}
	
	// joint_1
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_1;
    PID_Motor_parameter_Struct.angle.p =1;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d =10;
    PID_Motor_parameter_Struct.angle.max_out =10;
    PID_Motor_parameter_Struct.angle.integral_limit = 0.6;

    PID_Motor_parameter_Struct.speed.p = 0.01;
    PID_Motor_parameter_Struct.speed.i = 0;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 0.05;
    PID_Motor_parameter_Struct.speed.integral_limit = 0.6;

    Motor_PID_Struct_Init(&Motor[joint_1], PID_Motor_parameter_Struct, init_status);
	}

	// EXCHANGE_PITCH
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = joint_2;
    PID_Motor_parameter_Struct.angle.p = 0.1;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 50;
    PID_Motor_parameter_Struct.angle.max_out =0.1;
    PID_Motor_parameter_Struct.angle.integral_limit = 0.6;

    PID_Motor_parameter_Struct.speed.p = 1;
    PID_Motor_parameter_Struct.speed.i = 0;
    PID_Motor_parameter_Struct.speed.d = 0;
    PID_Motor_parameter_Struct.speed.max_out = 0.2;
    PID_Motor_parameter_Struct.speed.integral_limit = 0.6;

    Motor_PID_Struct_Init(&Motor[joint_2], PID_Motor_parameter_Struct, init_status);
	}

	// EXCHANGE_ROLL
	{
    PID_Motor_parameter_Struct.MOTOR_NAME = EXCHANGE_ROLL;
    PID_Motor_parameter_Struct.angle.p = 0;
    PID_Motor_parameter_Struct.angle.i = 0;
    PID_Motor_parameter_Struct.angle.d = 0;
    PID_Motor_parameter_Struct.angle.max_out = 0;
    PID_Motor_parameter_Struct.angle.integral_limit = 0;

    PID_Motor_parameter_Struct.speed.p = 60.0;
    PID_Motor_parameter_Struct.speed.i = 0.01;
    PID_Motor_parameter_Struct.speed.d = 0.1;
    PID_Motor_parameter_Struct.speed.max_out = 30000;
    PID_Motor_parameter_Struct.speed.integral_limit = 30000;

    Motor_PID_Struct_Init(&Motor[EXCHANGE_ROLL], PID_Motor_parameter_Struct, init_status);
	}
}

//模式角度初始化
void Motor_angle_init()
{
	// normal_angle
	{
		Motor[CHASSIS_FR].Angle.normal_angle = 0;
		Motor[UPRISE_L].Angle.normal_angle = 1000;//2500;MAX7000
		Motor[CLAMP_YAW].Angle.normal_angle = 30;
		Motor[CLAMP_PITCH].Angle.normal_angle = 130;
		Motor[CLAMP_ROLL].Angle.normal_angle = 0;
		Motor[SLIDE_L].Angle.normal_angle = -350;//-100MAX:-850
		Motor[joint_3_roll].Angle.smallisland_angle = 0;
		Motor[joint_1].Angle.normal_angle = 5;
		Motor[joint_2].Angle.normal_angle = 40;
		Motor[EXCHANGE_ROLL].Angle.normal_angle = 90;
		Motor[GIMBAL_YAW].Angle.normal_angle = NORMAL_LOOK;//62
	}

	// bigisland_straight_angle
	{
		Motor[CHASSIS_FR].Angle.bigisland_straight_angle = 0;
		Motor[UPRISE_L].Angle.bigisland_straight_angle = 2000;
		Motor[CLAMP_YAW].Angle.bigisland_straight_angle = 0;
		Motor[CLAMP_PITCH].Angle.bigisland_straight_angle = 0;
		Motor[CLAMP_ROLL].Angle.bigisland_straight_angle = 0;
		Motor[SLIDE_L].Angle.bigisland_straight_angle = -200;
		Motor[joint_3_roll].Angle.bigisland_straight_angle = -1350;//MAX-1950
		Motor[joint_1].Angle.bigisland_straight_angle = 0;
		Motor[joint_2].Angle.bigisland_straight_angle = 0;
		Motor[EXCHANGE_ROLL].Angle.bigisland_straight_angle = 0;
		Motor[GIMBAL_YAW].Angle.bigisland_straight_angle = MODE_LOOK;//62
	}

	// bigisland_slanted_angle
	{
		Motor[CHASSIS_FR].Angle.bigisland_slanted_angle = 0;
		Motor[UPRISE_L].Angle.bigisland_slanted_angle = 2000;
		Motor[CLAMP_YAW].Angle.bigisland_slanted_angle = 80;
		Motor[CLAMP_PITCH].Angle.bigisland_slanted_angle = 10;
		Motor[CLAMP_ROLL].Angle.bigisland_slanted_angle = 0;
		Motor[SLIDE_L].Angle.bigisland_slanted_angle = -250;
		Motor[joint_3_roll].Angle.bigisland_slanted_angle = -850;
		Motor[joint_1].Angle.bigisland_slanted_angle = 0;
		Motor[joint_2].Angle.bigisland_slanted_angle = 0;
		Motor[EXCHANGE_ROLL].Angle.bigisland_slanted_angle = 0;
		Motor[GIMBAL_YAW].Angle.bigisland_slanted_angle = MODE_LOOK;//62
	}

	//smallisland_angle
	{
		Motor[CHASSIS_FR].Angle.smallisland_angle = 0;
		Motor[UPRISE_L].Angle.smallisland_angle = 2500;//1300; //1000;
		Motor[CLAMP_YAW].Angle.smallisland_angle = 0;
		Motor[CLAMP_PITCH].Angle.smallisland_angle = -90;
		Motor[CLAMP_ROLL].Angle.smallisland_angle = 0;
		Motor[SLIDE_L].Angle.smallisland_angle = -80;//300
		Motor[joint_3_roll].Angle.smallisland_angle = 0;
		Motor[joint_1].Angle.smallisland_angle = 0;
		Motor[joint_2].Angle.smallisland_angle = test_angle;
		Motor[EXCHANGE_ROLL].Angle.smallisland_angle = 0;
		Motor[GIMBAL_YAW].Angle.smallisland_angle = MODE_LOOK;//62
	}
	//smallisland_angle_2
	{
		Motor[CHASSIS_FR].Angle.smallisland_angle_2 = 0;
		Motor[UPRISE_L].Angle.smallisland_angle_2 = 1000;//1300; //1000;
		Motor[CLAMP_YAW].Angle.smallisland_angle_2 = 0;
		Motor[CLAMP_PITCH].Angle.smallisland_angle_2 = -90;
		Motor[CLAMP_ROLL].Angle.smallisland_angle_2 = 0;
		Motor[SLIDE_L].Angle.smallisland_angle_2 = -100;//300
		Motor[joint_3_roll].Angle.smallisland_angle_2 = 0;
		Motor[joint_1].Angle.smallisland_angle_2 = 0;
		Motor[joint_2].Angle.smallisland_angle_2 = 200;
		Motor[EXCHANGE_ROLL].Angle.smallisland_angle_2 = 0;
		Motor[GIMBAL_YAW].Angle.smallisland_angle_2 = MODE_LOOK;//62
	}
	// ground_angle
	{
		Motor[CHASSIS_FR].Angle.ground_angle = 0;
		Motor[UPRISE_L].Angle.ground_angle = 500;
		Motor[CLAMP_YAW].Angle.ground_angle = 0;
		Motor[CLAMP_PITCH].Angle.ground_angle = 105;
		Motor[CLAMP_ROLL].Angle.ground_angle = 0;
		Motor[SLIDE_L].Angle.ground_angle = 0;
		Motor[joint_3_roll].Angle.ground_angle = -650;//-650
		Motor[joint_1].Angle.ground_angle = 0;
		Motor[joint_2].Angle.ground_angle = 0;
		Motor[EXCHANGE_ROLL].Angle.ground_angle = 0;
		Motor[GIMBAL_YAW].Angle.ground_angle = MODE_LOOK;//62
	}

	// exchange_angle
	{
		Motor[CHASSIS_FR].Angle.exchange_angle = 0;
		Motor[UPRISE_L].Angle.exchange_angle = 0;
		Motor[CLAMP_YAW].Angle.exchange_angle = -70;
		Motor[CLAMP_PITCH].Angle.exchange_angle = 0;
		Motor[CLAMP_ROLL].Angle.exchange_angle = 0;
		Motor[SLIDE_L].Angle.exchange_angle = -500;
		Motor[joint_3_roll].Angle.exchange_angle = 0;
		Motor[joint_1].Angle.exchange_angle = 0;
		Motor[joint_2].Angle.exchange_angle = 0;
		Motor[EXCHANGE_ROLL].Angle.exchange_angle = 0;
		Motor[GIMBAL_YAW].Angle.exchange_angle = MODE_LOOK;//62
	}

	// store_angle
	{
		Motor[CHASSIS_FR].Angle.store_angle = 0;
		Motor[UPRISE_L].Angle.store_angle = 4800;
		Motor[CLAMP_YAW].Angle.store_angle = 0;
		Motor[CLAMP_PITCH].Angle.store_angle = -90;
		Motor[CLAMP_ROLL].Angle.store_angle = 0;
		Motor[SLIDE_L].Angle.store_angle = 90;
		Motor[joint_3_roll].Angle.store_angle = -200;
		Motor[joint_1].Angle.store_angle = 0;
		Motor[joint_2].Angle.store_angle = -2400;
		Motor[EXCHANGE_ROLL].Angle.store_angle = 180;
		Motor[GIMBAL_YAW].Angle.store_angle = MODE_LOOK;//62
	}

	// exchange_pick_angle
	{
		Motor[CHASSIS_FR].Angle.exchange_pick_angle = 0;
		Motor[UPRISE_L].Angle.exchange_pick_angle = 4800;
		Motor[CLAMP_YAW].Angle.exchange_pick_angle = 0;
		Motor[CLAMP_PITCH].Angle.exchange_pick_angle = -90;
		Motor[CLAMP_ROLL].Angle.exchange_pick_angle = 0;
		Motor[SLIDE_L].Angle.exchange_pick_angle = 90;
		Motor[joint_3_roll].Angle.exchange_pick_angle = -200;
		Motor[joint_1].Angle.exchange_pick_angle = 0;
		Motor[joint_2].Angle.exchange_pick_angle = -2400;
		Motor[EXCHANGE_ROLL].Angle.exchange_pick_angle = 180;
		Motor[GIMBAL_YAW].Angle.exchange_pick_angle = MODE_LOOK;//62
	}
	{
		Motor[CHASSIS_FR].Angle.check_angle = 0;
		Motor[UPRISE_L].Angle.check_angle = 6000;
		Motor[CLAMP_YAW].Angle.check_angle = 0;
		Motor[CLAMP_PITCH].Angle.check_angle = 0;
		Motor[CLAMP_ROLL].Angle.check_angle = 0;
		Motor[SLIDE_L].Angle.check_angle = 0;
		Motor[joint_3_roll].Angle.check_angle = 0;
		Motor[joint_1].Angle.check_angle = 0;
		Motor[joint_2].Angle.check_angle = 0;
		Motor[EXCHANGE_ROLL].Angle.check_angle = 0;
		Motor[GIMBAL_YAW].Angle.check_angle = MODE_LOOK;//62
	}
}

void Motor_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	uint8_t ID;
	
	while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
								(uint32_t) INFO_GET_MOTOR_SIGNAL, 
								(uint32_t *)&Signal, 
								(TickType_t) portMAX_DELAY );
		if(STAUS ==pdTRUE)
		{

			
			if(Motor_init_state==INIT_NEVER)//未配置参数则进行参数配置
			{
				Motor_base_init();
				Motor_init_state=INIT_DONE;
			}
			for(ID=0;ID<MOTOR_NUMBER;ID++)//舵机计算
			{
				if(Whether_Brushless_Motor(Motor[ID])==0)
				{
					Motor_Servo_handler(ID);
				}
				
			}
			
	    
			if((Signal & INFO_GET_MOTOR_SIGNAL) && chassis_mode != CHASSIS_RELEASE)
			{
				/******************正式开始*****************************/
				if(All_Offset_Angle_init_state==0)
				{

			
					
					All_Offset_Angle_init_state=Motor_offset_angle_init();
				 
				}//判断
				
				else//堵转结束
				{
					clamp_angle_handle();
					for(ID=12;ID<14;ID++)//正式运行
					{
					if((Motor[ID].MOTOR_NAME>CHASSIS_BR) && (Whether_Brushless_Motor(Motor[ID])))//底盘没改，不在这里进行计算
						{
			
							/******************速度角度环切换计算*****************************/
							if(Motor[ID].MOTOR_TYPE!=go_8010)
						{
							Motor[ID].Brushless.Speed_or_Angle_flag=ANGLE_MODE;
							if(Motor[ID].Brushless.Angle_to_Speed_mode && All_Offset_Angle_init_state)
							{
								Motor[ID].Brushless.Speed_or_Angle_flag=SPEED_MODE;
								if(Motor[ID].Brushless.angle_ref-Motor[ID].Brushless.angle_fdb > 100)
								{
									Motor[ID].Brushless.spd_ref=  Motor[ID].Brushless.speed;
								}
								else if(Motor[ID].Brushless.angle_ref-Motor[ID].Brushless.angle_fdb < 100)
								{
									Motor[ID].Brushless.spd_ref= -Motor[ID].Brushless.speed;
								}
								else Motor[ID].Brushless.Speed_or_Angle_flag=ANGLE_MODE; 
							}
						}
						else
						{
							//Motor[ID].Brushless.Speed_or_Angle_flag=SPEED_MODE;
						}
						
							Motor_pid_clac(ID);
							Motor_current_into_CAN(ID);
						
						}//底盘去除括号非无刷电机去除括号
					}//循环括号
				}//判断括号
				
				
				/******************主体结束*****************************/
			}
				xTaskGenericNotify((TaskHandle_t) can_msg_send_Task_Handle, 
													 (uint32_t) INFO_SEND_MOTOR_SIGNAL, 
													 (eNotifyAction) eSetBits, 
													 (uint32_t *)NULL );
				xTaskGenericNotify((TaskHandle_t) comm_8010_task_Handle, 
													 (uint32_t) INFO_SEND_MOTOR_SIGNAL, 
													 (eNotifyAction) eSetBits, 
													 (uint32_t *)NULL );
				
				Motor_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    					
		}
	}
}


void Motor_base_init_copy(uint8_t low,uint8_t hight)
{
	uint8_t ID;
	uint8_t count=0;
	for(ID=low+1;ID<=hight;ID++)
	{
		memcpy(&Motor[ID],&Motor[low],sizeof(Motor[low]));
		if(Whether_Brushless_Motor(Motor[ID]))
		{
			count++;
			Motor[ID].MOTOR_NAME=(motor_name_status)ID;//改名
			Motor[ID].Brushless.ESC_ID=Motor[low].Brushless.ESC_ID+count;//改电调ID
		}
	}//复制
	
}

//对速度取反，并且遍历所有模式角度
void Motor_base_init_reversal(uint8_t ID)
{
	uint32_t first_mode_angle_adress;
	int16_t reversal_angle;
	uint8_t i;
	first_mode_angle_adress=(uint32_t)&Motor[ID].Angle.normal_angle;
	
	Motor[ID].Brushless.offset_angle_init_speed=-Motor[ID].Brushless.offset_angle_init_speed;
	for(i=0;i<MODE_ANGLE_NUMBER;i++)
	{
		reversal_angle=*(int16_t*)first_mode_angle_adress;
		*(int16_t*)first_mode_angle_adress=-reversal_angle;
		first_mode_angle_adress++;
		first_mode_angle_adress++;
	}
	
}

void Motor_PID_Struct_Init(motor_t *Motor_recieve,pid_motor_parameter_t parameter_Struct,INIT_STATUS init_status)
{
	PID_Struct_Init(&Motor_recieve->Brushless.angle_pid,parameter_Struct.angle.p,parameter_Struct.angle.i,parameter_Struct.angle.d,
		parameter_Struct.angle.max_out,parameter_Struct.angle.integral_limit,init_status);
	PID_Struct_Init(&Motor_recieve->Brushless.speed_pid,parameter_Struct.speed.p,parameter_Struct.speed.i,parameter_Struct.speed.d,
		parameter_Struct.speed.max_out,parameter_Struct.speed.integral_limit,init_status);
}

uint8_t Motor_offset_angle_init(void)
{
	uint8_t ID;
	uint8_t all_init_state;
	all_init_state=1;
	for(ID=0;ID<MOTOR_NUMBER;ID++)//堵转初始化
	{
		if(Motor[ID].MOTOR_NAME>CHASSIS_BR && Whether_Brushless_Motor(Motor[ID]))
		{
			if(Motor[ID].Brushless.offset_angle_init_flag==0)//底盘没改，不在这里进行计算
			{
				if(Motor[ID].MOTOR_TYPE==M3508||Motor[ID].MOTOR_TYPE==M2006)
				{
					Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.offset_angle_init_speed;
				}
				else if(Motor[ID].MOTOR_TYPE==M6020) 
				{
					if((Motor[ID].Brushless.ecd_fdb - 1679) > 0)
						Motor[ID].Brushless.spd_ref=-Motor[ID].Brushless.offset_angle_init_speed;
					else if((Motor[ID].Brushless.ecd_fdb - 1679) < 0)
						Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.offset_angle_init_speed;
					else Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;
				}
			
	




				
			if(Motor[ID].MOTOR_TYPE==go_8010)
			{
				Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;
				if(	Motor[ID].Angle.offset_angle==Motor[ID].Brushless.angle_fdb)
				{
				Motor[ID].Brushless.offset_angle_init_flag = 1;
				Motor[ID].Brushless.angle_ref=Motor[ID].Angle.offset_angle;
				}
			}
			else if(Motor[ID].MOTOR_TYPE!=go_8010)
			{
				if((fabs(Motor[ID].Brushless.spd_ref) - fabs(Motor[ID].Brushless.spd_fdb)) > 0.6*fabs(Motor[ID].Brushless.spd_ref))//判断是否堵转（占比越大条件越苛刻）
				{
					Motor[ID].Brushless.err_count++;
					if(Motor[ID].Brushless.err_count>200)
					{//达到堵转
						Motor[ID].Brushless.offset_angle_init_flag = 1;
						Motor[ID].Angle.offset_angle=Motor[ID].Brushless.angle_fdb;
						Motor[ID].Brushless.spd_ref=0;
						Motor[ID].Brushless.err_count=0;
					}
				}
			}
//				else
//				{
//					Motor[ID].Brushless.spd_ref=0;
//					Motor[ID].Brushless.err_count=0;
//				}
			}//未堵转化完成
			//motor_8010_speed_get_limit(ID);
			Motor_pid_clac(ID);
			Motor_current_into_CAN(ID);
			if(Motor[ID].Brushless.offset_angle_init_flag==0) all_init_state=0;//任意电机初始化未完成
		}//选中控制电机
	}//循环

	return all_init_state;
}

void Motor_Servo_handler(uint8_t ID)
{
	uint32_t TIM_out;
	TIM_out=((2000/Motor[ID].Servo.Rotation_range)*Motor[ID].Servo.angle_ref)+500;
	switch(Motor[ID].Servo.Compare)
	{
		case 1:
		{
			TIM_SetCompare1(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 2:
		{
			TIM_SetCompare2(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 3:
		{
			TIM_SetCompare3(Motor[ID].Servo.TIM,TIM_out);
		}break;
		case 4:
		{
			TIM_SetCompare4(Motor[ID].Servo.TIM,TIM_out);
		}break;
	}
}

void Motor_pid_clac(uint8_t ID)
{
	if(Motor[ID].Brushless.Speed_or_Angle_flag==ANGLE_MODE)
	{
		
		pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
		Motor[ID].Brushless.spd_ref=Motor[ID].Brushless.angle_pid.out;
		pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,Motor[ID].Brushless.spd_ref);
		Motor[ID].Brushless.current_send=Motor[ID].Brushless.speed_pid.out;
	}
	else if(Motor[ID].Brushless.Speed_or_Angle_flag==SPEED_MODE)
	{
		Motor[ID].Brushless.current_send=pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,Motor[ID].Brushless.spd_ref);
	}
	
	if(Motor[ID].MOTOR_TYPE==go_8010)
	{
		
		pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
		Motor[ID].Brushless.current_send=Motor[ID].Brushless.angle_pid.out;
		
//	  test_ref[ID] = pid_calc(&Motor[ID].Brushless.angle_pid,Motor[ID].Brushless.angle_fdb,Motor[ID].Brushless.angle_ref);
//		pid_calc(&Motor[ID].Brushless.speed_pid,Motor[ID].Brushless.spd_fdb,test_ref[ID]);
//		Motor[ID].Brushless.current_send=Motor[ID].Brushless.angle_pid.out;
//		test_angle=0.1*Motor[ID].Brushless.speed_pid.out;
	}
 
 
//	if(Motor[ID].MOTOR_TURN_FLAG)
//	{
//		Motor[ID].Brushless.current_send=-Motor[ID].Brushless.current_send;
//	}
}

void Motor_current_into_CAN(uint8_t ID)
{
	switch(Motor[ID].Brushless.CAN_ID)
	{
		case 1:
		{
			memcpy(&CAN1_current[Motor[ID].Brushless.ESC_ID],&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
		}break;
		case 2:
		{
			memcpy(&CAN2_current[Motor[ID].Brushless.ESC_ID],&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
		}break;
	}
//	
		if(Motor[ID].MOTOR_TYPE==go_8010)
		{
			switch(Motor[ID].Brushless.GO_ID)
		{
			case GO_8010_1:
			{
			memcpy(&send_current_8010_1,&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));			
			memcpy(&send_ref_angle_8010_1,&Motor[ID].Brushless.angle_ref,sizeof(Motor[ID].Brushless.angle_ref));
			memcpy(&send_fdb_angle_8010_1,&Motor[ID].Brushless.angle_fdb,sizeof(Motor[ID].Brushless.angle_fdb));	
				
			}break;
			case 3:
			{
				memcpy(&send_current_8010_2,&Motor[ID].Brushless.current_send,sizeof(Motor[ID].Brushless.current_send));
				memcpy(&send_ref_angle_8010_2,&Motor[ID].Brushless.angle_ref,sizeof(Motor[ID].Brushless.angle_ref));
			}break;
		}
		
	}
	
	
}

uint8_t Whether_Brushless_Motor(motor_t Motor)
{
	switch(Motor.MOTOR_TYPE)
	{
		case M6020:
		case M3508:
		case M2006:
		case go_8010:
			return 1;
	default:
	return 0;
	}
}












