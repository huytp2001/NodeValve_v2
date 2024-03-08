#include "process.h"
#include "lora.h"
#include "addr.h"
#include "valve.h"
#include "flash.h"

void v_process_ctr_frame(stru_frame_t *stru_frame_in)
{
	uint8_t au8_frame_out[FRAME_MAX_SIZE];
	v_frame_build(stru_frame_in, FRAME_TYPE_ACK_PREV, au8_frame_out);
	v_lora_send_frame(u8_addr_high, u8_addr_prev, u8_addr_channel, au8_frame_out, FRAME_ACK_SIZE);
	if (!b_is_end_node(stru_frame_in->u8_routing_val))
	{
		v_frame_build(stru_frame_in, FRAME_TYPE_CTR, au8_frame_out);
		v_lora_send_frame(u8_addr_high, u8_addr_next, u8_addr_channel, au8_frame_out, FRAME_CTR_SIZE);
	}
	else
	{
		HAL_Delay(ENDNODE_DELAY);
		v_frame_build(stru_frame_in, FRAME_TYPE_REP, au8_frame_out);
		v_lora_send_frame(u8_addr_high, u8_addr_prev, u8_addr_channel, au8_frame_out, FRAME_CTR_SIZE);
	}
	if (b_valve_check(stru_frame_in->u8_en_state_val)) v_valve_control_u32((uint32_t)stru_frame_in->u32_state_val);
	v_timer_reset();
}

void v_process_rep_frame(stru_frame_t *stru_frame_in)
{
	uint8_t au8_frame_out[FRAME_MAX_SIZE];
	v_frame_build(stru_frame_in, FRAME_TYPE_ACK_NEXT, au8_frame_out);
	v_lora_send_frame(u8_addr_high, u8_addr_next, u8_addr_channel, au8_frame_out, FRAME_ACK_SIZE);
	v_frame_build(stru_frame_in, FRAME_TYPE_REP, au8_frame_out);
	v_lora_send_frame(u8_addr_high, u8_addr_prev, u8_addr_channel, au8_frame_out, FRAME_CTR_SIZE);
	v_timer_reset();
}

void v_process_wait_ctr(void)
{
	if (u16_timer_get() > 10 * WAITING_CTR_TIMEOUT_SEC) 
	{
		v_timer_reset();
		v_valve_control_u32((uint32_t)(u8_flash_read(0) & 0x0F));
		HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_RESET);
		v_Mcu_Sleep();
		v_timer_reset();
		HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_SET);
	}
}

void v_process_wait_rep(void)
{
	if (u16_timer_get() > 10 * WAITING_CTR_TIMEOUT_SEC)
	{
		v_timer_reset();
		HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_RESET);
		v_Mcu_Sleep();
		v_timer_reset();
		HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_SET);
	}
}

void v_process_wait_ack(stru_frame_t *stru_frame_in, bool b_direction)
{
	if (u16_timer_get() > 10 * WAITING_ACK_TIMEOUT_SEC)
	{
		uint8_t au8_frame_out[FRAME_MAX_SIZE];
		if (b_direction) // ACK NEXT (FOR REP FRAME)
		{
			stru_frame_in->u8_routing_val ^= (0x01 << (u8_addr_prev - 1));
			v_addr_continuous_setup(stru_frame_in->u8_routing_val);
			v_frame_build(stru_frame_in, FRAME_TYPE_REP, au8_frame_out);
			v_lora_send_frame(u8_addr_low, u8_addr_prev, u8_addr_channel, au8_frame_out, FRAME_CTR_SIZE);
			v_timer_reset();
		}
		else // ACK PREV (FOR CTR FRAME)
		{
			stru_frame_in->u8_routing_val ^= (0x01 << (u8_addr_next - 1));
			v_addr_continuous_setup(stru_frame_in->u8_routing_val);
			v_frame_build(stru_frame_in, FRAME_TYPE_CTR, au8_frame_out);
			v_lora_send_frame(u8_addr_low, u8_addr_next, u8_addr_channel, au8_frame_out, FRAME_CTR_SIZE);
			v_timer_reset();
		}
	}
}




