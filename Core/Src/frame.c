#include "frame.h"
#include "addr.h"
#include "battery.h"
#include "valve.h"

uint8_t au8_frame_data[FRAME_MAX_SIZE];
uint8_t u8_frame_size;

void v_frame_set_data(uint8_t* frame_buf, uint8_t frame_size)
{
	memcpy(au8_frame_data, frame_buf, frame_size);
	u8_frame_size = frame_size;
}

e_frame_err_t e_frame_check(void)
{
	if (au8_frame_data[0] != FRAME_CTR_SIZE && au8_frame_data[0] != FRAME_ACK_SIZE)
	{
		return FRAME_SIZE_ERR; 
	}
	uint16_t u16_sum = 0;
	for (uint8_t i = 1; i < u8_frame_size-2; i++)
	{
		u16_sum += au8_frame_data[i];
	}
	uint16_t u16_sum_data = ((uint16_t)au8_frame_data[u8_frame_size - 2] << 8) | (uint16_t)au8_frame_data[u8_frame_size - 1];
	if (u16_sum != u16_sum_data)
	{
		return FRAME_CHECKSUM_ERR;
	}
	if ( au8_frame_data[2] != u8_addr_high || au8_frame_data[3] != u8_addr_low || au8_frame_data[4] != u8_addr_channel)
	{
		return FRAME_ADDR_ERR;
	}
	return FRAME_OK;
}

void v_frame_read(stru_frame_t* stru_frame_out)
{
	stru_frame_out->u8_lenght_frame = au8_frame_data[0];
	stru_frame_out->u8_frame_typeID = au8_frame_data[1];
	stru_frame_out->u8_high_addr = au8_frame_data[2];
	stru_frame_out->u8_low_addr = au8_frame_data[3];
	stru_frame_out->u8_chan_addr = au8_frame_data[4];
	switch (au8_frame_data[1])
	{
		case FRAME_TYPE_CTR:
		{
			stru_frame_out->u8_routing_val = au8_frame_data[5];
			stru_frame_out->u8_en_state_val = au8_frame_data[6];
			stru_frame_out->u32_state_val = ((uint32_t)au8_frame_data[7]<<24)|((uint32_t)au8_frame_data[8]<<16)|((uint32_t)au8_frame_data[9]<<8)|((uint32_t)au8_frame_data[10]);
		}
		break;
		case FRAME_TYPE_REP:
		{
			stru_frame_out->u8_routing_val = au8_frame_data[5];
			stru_frame_out->u8_battery_val = au8_frame_data[6];
			stru_frame_out->u32_state_val = ((uint32_t)au8_frame_data[7]<<24)|((uint32_t)au8_frame_data[8]<<16)|((uint32_t)au8_frame_data[9]<<8)|((uint32_t)au8_frame_data[10]);
		}
		break;
	}
	stru_frame_out->u16_checksum = ((uint16_t)au8_frame_data[au8_frame_data[0]-2]<<8)|((uint16_t)au8_frame_data[au8_frame_data[0]-1]); 
}

void v_frame_build(stru_frame_t *stru_frame_in, e_frame_type_t e_frame_type, uint8_t au8_frame_out[]) 
{
	memset(au8_frame_out, 0x00, FRAME_CTR_SIZE);
	au8_frame_out[1] = e_frame_type;
	au8_frame_out[2] = u8_addr_high;
	au8_frame_out[4] = u8_addr_channel;
	switch (e_frame_type)
	{
		case FRAME_TYPE_CTR:
		{
			au8_frame_out[0] = FRAME_CTR_SIZE;
			au8_frame_out[3] = u8_addr_next;
			au8_frame_out[5] = stru_frame_in->u8_routing_val;
			au8_frame_out[6] = stru_frame_in->u8_en_state_val;
			au8_frame_out[7] = (uint8_t)(stru_frame_in->u32_state_val>>24)&0xFF;
			au8_frame_out[8] = (uint8_t)(stru_frame_in->u32_state_val>>16)&0xFF;
			au8_frame_out[9] = (uint8_t)(stru_frame_in->u32_state_val>>8)&0xFF;
			au8_frame_out[10] = (uint8_t)(stru_frame_in->u32_state_val)&0xFF;
		}
		break;
		case FRAME_TYPE_REP:
		{
			au8_frame_out[0] = FRAME_CTR_SIZE;
			au8_frame_out[3] = u8_addr_prev;
			au8_frame_out[5] = stru_frame_in->u8_routing_val;
			au8_frame_out[6] = stru_frame_in->u8_battery_val | (0x01 << (u8_addr_low - 1));
			au8_frame_out[7] = (uint8_t)(stru_frame_in->u32_state_val>>24)&0xFF;
			au8_frame_out[8] = (uint8_t)(stru_frame_in->u32_state_val>>16)&0xFF;
			au8_frame_out[9] = (uint8_t)(stru_frame_in->u32_state_val>>8)&0xFF;
			au8_frame_out[10] = (uint8_t)(stru_frame_in->u32_state_val)&0xFF;
		}
		break;
		case FRAME_TYPE_ACK_NEXT:
		{
			au8_frame_out[0] = FRAME_ACK_SIZE;
			au8_frame_out[3] = u8_addr_next;
		}
		break;
		case FRAME_TYPE_ACK_PREV:
		{
			au8_frame_out[0] = FRAME_ACK_SIZE;
			au8_frame_out[3] = u8_addr_prev;
		}
		break;
	}
	uint16_t u16_sum = 0;
	for (uint8_t i = 1; i < au8_frame_out[0]-2; i++) u16_sum += au8_frame_out[i];
	au8_frame_out[au8_frame_out[0]-2] = (uint8_t)(u16_sum>>8)&0xFF;
	au8_frame_out[au8_frame_out[0]-1] = (uint8_t)(u16_sum)&0xFF;
}



