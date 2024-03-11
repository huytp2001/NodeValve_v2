#ifndef VALVE_H
#define VALVE_H
#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#include "stdbool.h"

bool b_valve_check(uint8_t u8_en_state_val);
void v_valve_control_u32(uint32_t u32_new_state_val);
uint8_t u8_valve_get_state(void);

#ifdef __cplusplus
}
#endif
#endif

