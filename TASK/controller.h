#ifndef _controller_H
#define _controller_H

#include "judge_rx_data.h"

#define CONTROLLER_DATA_LENGTH 12
#define JUDGE_FORMAT_LENGTH 9
#define MODEL_MODE 0
#define GYROSOPE_MODE 1

#define AS5600_NUMBER 6

#define CONTROLLER_ALL_DATA_LENGTH CONTROLLER_DATA_LENGTH+JUDGE_FORMAT_LENGTH

/**************比率换算************************/
#define MODEL_X_MAX 771
#define MODEL_Y_MAX 518
#define MODEL_Z_MAX 766

#define EXCHANGE_X_MAX 550
#define EXCHANGE_Y_MAX 550
#define EXCHANGE_Z_MAX 5500

#define SMALL_MODEL_X_RATIO EXCHANGE_X_MAX/MODEL_X_MAX
#define SMALL_MODEL_Y_RATIO EXCHANGE_Y_MAX/MODEL_Y_MAX
#define SMALL_MODEL_Z_RATIO EXCHANGE_Z_MAX/MODEL_Z_MAX
#define SMALL_MODEL_PITCH_RATIO (25.5/1)

/**************比率换算************************/

typedef struct
{
	uint8_t cmd_id;
	
	int16_t pit_gyro_angle;
  int16_t roll_gyro_angle;
  int16_t yaw_gyro_angle;
	
	int16_t x_displacement;
	int16_t y_displacement;
	int16_t z_displacement;
}gyrosope_data_t;

typedef struct
{
	uint8_t cmd_id;
	uint8_t main_data[29];
}usart_data_t;

extern gyrosope_data_t gyrosope_data,gyrosope_past_data;
extern int16_t small_model_data[6];
extern int16_t model_out_angle[AS5600_NUMBER];
void controller_task(void);
#endif
