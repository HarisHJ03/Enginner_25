#ifndef _clamp
#define _clamp

#include "stdint.h"

//这部分没写好所以先写1，以后有需要再说
#define UPRISE_DISPLACEMENT_TO_ANGLE 1;// 角度/距离 的值，以毫米为单位
#define SLIDE_DISPLACEMENT_TO_ANGLE 1;// 角度/距离 的值，以毫米为单位


#define ACTIONING 0
#define ACTION_DONE 1
#define SECOND_ACTION_DONE 2
#define MAX_BOX_NUMBER 2

#define SLIDE_MAX_CHANGE 765

typedef struct
{
	float normal;
	float special;
	uint8_t flag;
}Motor_current_limit_t;

void clamp_angle_handle(void);

void normal_clmap_handler(void);
void big_island_stright_clamp_handler(void);
void big_island_slanted_clamp_handler(void);
void small_island_clamp_handler(void);
void clamp_ground_handler(void);
void exchange_handler(void);	
void check_handler(void);

uint8_t store_handler(void);
uint8_t pick_handler(void);

void Motor_change_mode_angle(int16_t* Mode_now,uint8_t low_ID,uint8_t hight_ID);

void Motor_uprise_angle_change(int16_t change_displacement);
void Motor_slide_angle_change(int16_t change_displacement);

extern uint8_t have_box_number;


static uint8_t Motor_current_limit_init(void);
static void Motor_current_limit_reset(void);
static void Motor_current_limit_handler(void);
#endif

