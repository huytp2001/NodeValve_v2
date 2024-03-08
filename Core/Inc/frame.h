#ifndef FRAME_H
#define FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FRAME_MAX_SIZE 13
#define FRAME_CTR_SIZE 13
#define FRAME_ACK_SIZE 7

typedef struct {
	uint8_t u8_lenght_frame;
	uint8_t u8_frame_typeID;
	uint8_t u8_high_addr;
	uint8_t u8_low_addr;
	uint8_t u8_chan_addr;
	uint8_t u8_routing_val;
	uint8_t u8_en_state_val;
	uint8_t u8_battery_val;
	uint16_t u16_checksum;
	uint32_t u32_state_val;
} stru_frame_t;

typedef enum {
	FRAME_OK,
	FRAME_SIZE_ERR,
	FRAME_CHECKSUM_ERR,
	FRAME_ADDR_ERR
} e_frame_err_t;

typedef enum {
	FRAME_TYPE_CTR = 0x43,
	FRAME_TYPE_ACK_PREV = 0x3E,
	FRAME_TYPE_REP = 0x52,
	FRAME_TYPE_ACK_NEXT = 0x3C
} e_frame_type_t;

typedef enum {
	WAITING_CTR,
	WAITING_REP,
	WAITING_NEXT_ACK,
	WAITING_PREV_ACK
} e_frame_waiting_t;

extern uint8_t au8_frame_data[FRAME_MAX_SIZE];
extern uint8_t u8_frame_size;

void v_frame_set_data(uint8_t* frame_buf, uint8_t frame_size);
e_frame_err_t e_frame_check(void);
void v_frame_read(stru_frame_t* stru_frame_out);
void v_frame_build(stru_frame_t *stru_frame_in, e_frame_type_t e_frame_type, uint8_t au8_frame_out[]);

#ifdef __cplusplus
}
#endif

#endif


