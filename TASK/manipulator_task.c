#include "manipulator_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "upraise_task.h"
#include "chassis_task.h"
#include "slide_task.h"
#include "remote_ctrl.h"
#include "remote_ctrl.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "bsp_can.h"
#include "sys_config.h"
#include "math.h"
#include "ramp.h"
//#include "keyboard.h"
#include "bsp_trigonometry.h"
#include "controller.h"

pid_parameter_t pid_manipulator_angle_parameter[6]={0};
pid_parameter_t pid_manipulator_spd_parameter[6]={0};

pid_t pid_manipulator[6]={0};
pid_t pid_manipulator_spd[6]={0};
manipulator_t manipulator[7];
manipulator_t manipulator_debug;

float manipulator0_total_angle;
int16_t manipulator_servo_total_angle;
int16_t manipulator_normal_total_angle;
int16_t manipulator_servo_total;
uint8_t manipulator_error[6];//判断堵转累计次数
int16_t chassis_x_angle;

void manipulator_pid_init(INIT_STATUS init_status)
{
	/*	0——兑换yaw  1——兑换pitch  2——兑换roll
			3——夹取yaw	4——夹取pitch	5——夹取roll
	*/
	if(init_status == INIT)
	{
		manipulator[0].speed=1000;
		manipulator[1].speed=4000;
		manipulator[2].speed=700;
		manipulator[3].speed=700;
		manipulator[4].speed=700;
		manipulator[5].speed=2000;
		
		manipulator[0].spd_ref=0;
		manipulator[1].spd_ref=0;
		manipulator[2].spd_ref=0;
		manipulator[3].spd_ref=0;
		manipulator[4].spd_ref=0;
		manipulator[5].spd_ref=0;

		manipulator[0].normal_angle=0;
		manipulator[1].normal_angle=-1700;//700;//-16->-346
		manipulator[2].normal_angle=135;//179->54
		manipulator[3].normal_angle=0;
		manipulator[4].normal_angle=130;//-97 ->-95
		manipulator[5].normal_angle=0;//-258->-69
		//manipulator[6].normal_angle=1240;//1900最低，1200中间，600最高 90度差666	2000
																		//1400
		manipulator[0].exchange_angle=0;
		manipulator[1].exchange_angle=300;
		manipulator[2].exchange_angle=0;
		manipulator[3].exchange_angle=0;
		manipulator[4].exchange_angle=0;
		manipulator[5].exchange_angle=-1000;
		//manipulator[6].exchange_angle=166;
		
		manipulator[0].store_angle=-196;
		manipulator[1].store_angle=40;
		manipulator[2].store_angle=170;
		manipulator[3].store_angle=0;
		manipulator[4].store_angle=1;//3
		manipulator[5].store_angle=0;
		//manipulator[6].store_angle=-370;//-170
		
		manipulator[0].exchange_pick_angle=-196;
		manipulator[1].exchange_pick_angle=40;
		manipulator[2].exchange_pick_angle=170;
		manipulator[3].exchange_pick_angle=1;
		manipulator[4].exchange_pick_angle=2;//6  4
		manipulator[5].exchange_pick_angle=0;
		//manipulator[6].exchange_pick_angle=-370;//-170
		
		manipulator[1].exchange_pick_angle_2=-10;
		manipulator[2].exchange_pick_angle_2=207;
		manipulator[3].exchange_pick_angle_2=0;
		manipulator[4].exchange_pick_angle_2=-90;//5
		manipulator[5].exchange_pick_angle_2=0;
		//manipulator[6].exchange_pick_angle_2=0;
		
		//         straight/slanted
		manipulator[0].bigisland_straight_angle=0;
		manipulator[1].bigisland_straight_angle=0;//-2200
		manipulator[2].bigisland_straight_angle=0;//增加上抬30
		manipulator[3].bigisland_straight_angle=0;
		manipulator[4].bigisland_straight_angle=0;
		manipulator[5].bigisland_straight_angle=-1200;
		//manipulator[6].bigisland_straight_angle=0;
		manipulator[1].bigisland_slanted_angle=90;//加大是放低
		manipulator[2].bigisland_slanted_angle=0;//加大是上抬
		manipulator[3].bigisland_slanted_angle=0;
		manipulator[4].bigisland_slanted_angle=-3;
		manipulator[5].bigisland_slanted_angle=0;
		
		manipulator[0].smallisland_angle=0;
		manipulator[1].smallisland_angle=0;//加大是放低
		manipulator[2].smallisland_angle=0;//绝对值减小是上抬
		manipulator[3].smallisland_angle=0;
		manipulator[4].smallisland_angle=0;
		manipulator[5].smallisland_angle=0;
		//manipulator[6].smallisland_angle=0;
		
		manipulator[0].ground_angle=0;
		manipulator[1].ground_angle=0;//
		manipulator[2].ground_angle=0;//
		manipulator[3].ground_angle=0;
		manipulator[4].ground_angle=90;
		manipulator[5].ground_angle=0;
		//manipulator[6].ground_angle=200;
		
		
		//manipulator[6].bigisland_slanted_angle=-666;
		
		manipulator[0].mode_angle=0;
		manipulator[1].mode_angle=0;
		manipulator[2].mode_angle=0;
		manipulator[3].mode_angle=0;
		manipulator[4].mode_angle=0;
		manipulator[5].mode_angle=0;
		//manipulator[6].mode_angle=0;
}

	static uint8_t ID;

	/*	0——兑换yaw  	1——兑换pitch  	2——兑换roll
			3——夹取yaw		4——夹取ptich			5——暂无，测试完再删除
	*/
	pid_manipulator_angle_parameter[0].p=60.0;//25
	pid_manipulator_angle_parameter[0].i=0.01;//0.01
	pid_manipulator_angle_parameter[0].d=0.1;
	pid_manipulator_angle_parameter[0].max_out=30000;
	pid_manipulator_angle_parameter[0].integral_limit=30000;
	
	pid_manipulator_spd_parameter[0].p=50.0;//70
	pid_manipulator_spd_parameter[0].i=0.01;//0.2
	pid_manipulator_spd_parameter[0].d=0.1;
	pid_manipulator_spd_parameter[0].max_out=30000;
	pid_manipulator_spd_parameter[0].integral_limit=30000;

	pid_manipulator_angle_parameter[1].p=15.0;
	pid_manipulator_angle_parameter[1].i=0.0;//0.01
	pid_manipulator_angle_parameter[1].d=0.0;//0.1
	pid_manipulator_angle_parameter[1].max_out=7000;
	pid_manipulator_angle_parameter[1].integral_limit=1000;
	
	pid_manipulator_spd_parameter[1].p=10.0;
	pid_manipulator_spd_parameter[1].i=0.0;
	pid_manipulator_spd_parameter[1].d=0.0;
	pid_manipulator_spd_parameter[1].max_out=1500;
	pid_manipulator_spd_parameter[1].integral_limit=6000;

	pid_manipulator_angle_parameter[2].p=25.0;//30+
	pid_manipulator_angle_parameter[2].i=0;
	pid_manipulator_angle_parameter[2].d=0;
	pid_manipulator_angle_parameter[2].max_out=15000;
	pid_manipulator_angle_parameter[2].integral_limit=6000;
	
	pid_manipulator_spd_parameter[2].p=30.0;//30
	pid_manipulator_spd_parameter[2].i=0.05;//0
	pid_manipulator_spd_parameter[2].d=0.05;//0
	pid_manipulator_spd_parameter[2].max_out=15000;
	pid_manipulator_spd_parameter[2].integral_limit=6000;

	

	pid_manipulator_angle_parameter[4].p=90.0;
	pid_manipulator_angle_parameter[4].i=0.01;
	pid_manipulator_angle_parameter[4].d=0.0;
	pid_manipulator_angle_parameter[4].max_out=7000;
	pid_manipulator_angle_parameter[4].integral_limit=500;
	

	pid_manipulator_spd_parameter[4].p=10.0;
	pid_manipulator_spd_parameter[4].i=0.0;
	pid_manipulator_spd_parameter[4].d=0.0;
	pid_manipulator_spd_parameter[4].max_out=14000;
	pid_manipulator_spd_parameter[4].integral_limit=500;

	pid_manipulator_angle_parameter[5].p=90.0;
	pid_manipulator_angle_parameter[5].i=0.0;
	pid_manipulator_angle_parameter[5].d=0.0;
	pid_manipulator_angle_parameter[5].max_out=7000;
	pid_manipulator_angle_parameter[5].integral_limit=500;
	
	pid_manipulator_spd_parameter[5].p=20.0;
	pid_manipulator_spd_parameter[5].i=0.01;
	pid_manipulator_spd_parameter[5].d=0.0;
	pid_manipulator_spd_parameter[5].max_out=4000;
	pid_manipulator_spd_parameter[5].integral_limit=1500;	
	
	for(ID=0;ID<=5;ID++)
	{
		PID_Struct_Init(&pid_manipulator[ID],pid_manipulator_angle_parameter[ID].p,pid_manipulator_angle_parameter[ID].i,pid_manipulator_angle_parameter[ID].d,
		pid_manipulator_angle_parameter[ID].max_out,pid_manipulator_angle_parameter[ID].integral_limit,init_status);
			
		PID_Struct_Init(&pid_manipulator_spd[ID],pid_manipulator_spd_parameter[ID].p,pid_manipulator_spd_parameter[ID].i,pid_manipulator_spd_parameter[ID].d,
		pid_manipulator_spd_parameter[ID].max_out,pid_manipulator_spd_parameter[ID].integral_limit,init_status);
	}
}

uint8_t manipulator_motor_angle(int8_t ID)//用于角度控制 （ID，需要相对常态增加/减少的角度）
{
//	manipulator[ID].angle_ref=manipulator[ID].normal_angle + kb_manipulator_adjust_angle[ID] + manipulator[ID].mode_angle;
//	manipulator[3].angle_ref+=gyrosope_data.yaw_gyro_angle;
//	manipulator[2].angle_ref+=gyrosope_data.pit_gyro_angle;
//	manipulator[5].angle_ref-=gyrosope_data.roll_gyro_angle;
	
	if((fabs(fabs(manipulator[ID].angle_ref) - fabs(moto_manipulator[ID].total_angle)) < 30))
	{
		manipulator[ID].spd_ref=pid_calc(&pid_manipulator[ID],manipulator[ID].angle_fdb,manipulator[ID].angle_ref);
		manipulator[ID].current=pid_calc(&pid_manipulator_spd[ID],manipulator[ID].spd_fdb,manipulator[ID].spd_ref);
	}
	else
	{
		if(manipulator[ID].angle_ref - moto_manipulator[ID].total_angle > 30)//100-60=40>10 -100+60=-40 <-10
		{
			if(ID==MANIPULATOR_6020_ID) manipulator[ID].current=30000;
			else manipulator_motor_speed(ID,+manipulator[ID].speed);
		}
		if(manipulator[ID].angle_ref - moto_manipulator[ID].total_angle <-30)
		{
			if(ID==MANIPULATOR_6020_ID) manipulator[ID].current=-30000;
			else manipulator_motor_speed(ID,-manipulator[ID].speed);
		}
	}

	return 0;
}

uint8_t manipulator_motor_speed(int8_t ID,int16_t speed)//用于速度控制	（ID，+1/-1表示两种旋转方向）
{	
	manipulator[ID].init_flag = 0;	
	manipulator[ID].spd_ref = speed;
	
  
	manipulator[ID].current=pid_calc(&pid_manipulator_spd[ID],manipulator[ID].spd_fdb,manipulator[ID].spd_ref);
	
	if((fabs(manipulator[ID].spd_ref) - fabs(manipulator[ID].spd_fdb)) > 200)//判断是否堵转（占比越大条件越苛刻）
	{
		manipulator_error[ID]++;
		if(manipulator_error[ID]>200)
		{
			manipulator[ID].init_flag = 1;
		}
	}
	return manipulator[ID].init_flag;
}

//void manipulator_test(uint8_t ID)//测试
//{
//	memcpy(((char *)&manipulator_debug),((char *)&manipulator[ID]), sizeof(manipulator[ID]));
//}
//uint8_t manipulator_motor_angle_and_speed(int8_t ID,int32_t target,int16_t speed)//串级控制和单环控制复用
//{
//	manipulator[ID].angle_ref=manipulator[ID].init_angle+target;
//	manipulator[ID].init_flag=0;
//		
//	if (fabs(fabs(manipulator[ID].angle_ref) - fabs(moto_manipulator[ID].total_angle)) < 10) manipulator[ID].init_flag=1;

//	if(manipulator[ID].init_flag == 0)
//	{
//		manipulator_motor_speed(ID,speed);
//	}
//	else
//	{
//		manipulator_motor_angle(ID,target);
//	}
//	return manipulator[ID].init_flag;
//}

	/*****一个判断吸盘朝向一键前进的函数，研发中*****/
//	uint8_t suction_advancing_flag=0;
//	void suction_advance(int16_t speed)
//	{
//		float angle_displacement_x=300.0;//9
//		float angle_displacement_y=377.0;
//		float angle_displacement_z=327.0;//待测

//		int16_t suction_pitch,suction_yaw;
//		float x,y,z;
//		//正面朝前是y轴方向，竖直向上是z轴方向
//		//定义正直朝前为0,上正下负，右正左负
//		//各移动一个矿石的距离记录电机角度变化值，该值在速度环控制同样需要
//		
//		//位移一个矿石下各轴电机角度变化值
//		
//		suction_advancing_flag=1;
//		
//		//计算最前面的舵机的角度
//		manipulator_servo_total_angle=(manipulator_servo_total-500)*(180/2000);
//		manipulator_normal_total_angle=(manipulator[6].normal_angle-500)*(180/2000);
//		
//		//pitch与yaw计算
//		suction_pitch =	+(moto_manipulator[2].total_angle - manipulator[2].normal_angle)
//						+((manipulator_servo_total_angle - manipulator_normal_total_angle)*fast_cos(moto_manipulator[5].total_angle - manipulator[5].normal_angle));
//		suction_yaw 	= +(moto_manipulator[3].total_angle - manipulator[3].normal_angle) + (moto_manipulator[4].total_angle - manipulator[4].normal_angle)
//										+ ((manipulator_servo_total_angle - manipulator_normal_total_angle)*fast_sin(moto_manipulator[5].total_angle - manipulator[5].normal_angle));
//		
//		//换算xyz三轴
//		z=fast_sin(suction_pitch);
//		x=fast_sin(suction_yaw);
//		y=fast_cos(suction_yaw)*fast_cos(suction_pitch);
//		//转换
//		x*=speed*angle_displacement_x;
//		y*=speed*angle_displacement_y;
//		z*=speed*angle_displacement_z;

//		exchange_chassis_spd=(int16_t)x;
//		slide.spd_ref[0]=-(int16_t)y;
//		slide.spd_ref[1]=+(int16_t)y;
//		upraise.spd_ref[0]=-(int16_t)z;
//		upraise.spd_ref[1]=+(int16_t)z;
//	}

int Servo_convert(int MAX_angle,int Target_angle )
{
	int resulat;
	resulat=(2000/MAX_angle)*Target_angle + 500;
	return resulat;
}



