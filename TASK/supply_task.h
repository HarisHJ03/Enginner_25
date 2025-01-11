#ifndef _supply_task_H
#define _supply_task_H


#include "stm32f4xx.h"

typedef struct
{  
  uint8_t supply_cmd1;
	uint8_t supply_cmd2;
	/*¿ØÖÆ¾ÈÔ®ÏÂµÄÌ§Éı×´Ì¬*/
	uint8_t upraise_updown_flag;
  
}supply_t;

extern supply_t supply;
extern uint32_t supply_action_times;
extern uint8_t supply_ahead_finish ;
extern uint8_t supply_behind_state;
extern uint32_t  supply_continuous_time;       ;

void supply_task(void *parm);
void supply_param_init(void);
void supply_init_handler(void);
void supply_to_hero_handler(void);
void supply_to_infantry_handler(void);

#endif


