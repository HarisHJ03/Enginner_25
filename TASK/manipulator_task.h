#ifndef _manipulator_H
#define _manipulator_H

#include "pid.h"
#include "fuzzy_pid.h"


uint8_t manipulator_motor_angle(int8_t ID);
uint8_t manipulator_motor_speed(int8_t ID,int16_t speed);
void manipulator_pid_init(INIT_STATUS init_status);
void suction_advance(int16_t speed);
void manipulator_test(uint8_t ID);
int Servo_convert(int MAX_angle,int Target_angle );
//uint8_t manipulator_motor_angle_and_speed(int8_t ID,int32_t target,int16_t speed);

typedef struct
{
  uint8_t state;
	uint8_t init_flag;
	
  int16_t spd_ref;//速度环目标
  int16_t spd_fdb;//速度环get
	
	int16_t current;//电流
	int16_t speed;
  
  int32_t angle_ref;//角度环目标
  int32_t angle_fdb;//角度环get
  
  int16_t init_angle;
	int16_t exchange_angle;
	int16_t exchange_pick_angle;
	int16_t store_angle;
	int16_t exchange_pick_angle_2;
  int16_t bigisland_straight_angle;
	int16_t bigisland_slanted_angle;	
	int16_t smallisland_angle;
  int16_t normal_angle;
	int16_t ground_angle;
	
	int16_t mode_angle;
}manipulator_t;//不同电机之间的状态并不相同，不似抬升等电机是对称关系的，因此全部状态都整了个数组

extern manipulator_t manipulator_debug;
extern manipulator_t manipulator[7];
extern uint8_t manipulator_error[6];
extern float manipulator0_total_angle;
extern pid_t pid_manipulator[6];
extern pid_t pid_manipulator_spd[6];
extern int16_t chassis_x_angle;
extern int16_t manipulator_servo_total;

extern manipulator_t slide_24[SLIDE_NUMBER];
#endif
