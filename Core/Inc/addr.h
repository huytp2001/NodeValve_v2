#ifndef ADDR_H
#define ADDR_H
#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"

#define ADDR_CHAN_868MHZ 0x06
#define ADDR_CHAN_915MHZ 0x35

extern uint8_t u8_addr_low;
extern uint8_t u8_addr_high;
extern uint8_t u8_addr_channel;
extern uint8_t u8_addr_next;
extern uint8_t u8_addr_prev;

void v_addr_setup(void);
void v_addr_continuous_setup(uint8_t u8_routing);
bool b_is_end_node(uint8_t u8_routing);
bool b_is_start_node(uint8_t u8_routing);
	
#ifdef __cplusplus
}
#endif
#endif

