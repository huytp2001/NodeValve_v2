#include "lora.h"
#include "addr.h"

void v_lora_set_mode(e_lora_mode_t e_mode) 
{
	switch(e_mode)
	{
		case MODE_NORMAL:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
		}
		break;
	  case MODE_WAKEUP:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
		}
		break;
		case MODE_POWERDOWN:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
		}
		break;
		case MODE_PROGRAM:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
		}
		break;
		default: 
			break;
	}
	HAL_Delay(PIN_RECOVER);
}

uint8_t u8_lora_set_sped(uint8_t u8_parity_2bit, uint8_t u8_uart_bps_3bit, uint8_t u8_air_bps_3bit)
{
	uint8_t u8_sped = ((u8_parity_2bit & 0x03) << 6) | ((u8_uart_bps_3bit & 0x07) << 3) | (u8_air_bps_3bit & 0x07);
	return u8_sped;
}

uint8_t u8_lora_set_option(bool b_fm_bit, bool b_io_driver_bit, uint8_t u8_wakeup_t_3bit, bool b_fec_bit, uint8_t u8_opt_2bit)
{
	uint8_t u8_option = ((b_fm_bit & 0x01) << 7) | ((b_io_driver_bit & 0x01) << 6) | ((u8_wakeup_t_3bit & 0x07) << 3) | ((b_fec_bit & 0x01) << 2) | (u8_opt_2bit & 0x03);
	return u8_option;
}

void v_lora_save_params(uint8_t u8_save_mode, uint8_t u8_sped, uint8_t u8_options)
{
	v_lora_set_mode(MODE_PROGRAM);
	uint8_t au8_params[6] = {u8_save_mode, u8_addr_high, u8_addr_low, u8_sped, u8_addr_channel, u8_options};
	v_uart2_transmit(au8_params, 6);
	HAL_Delay(PIN_RECOVER);
	v_lora_set_mode(MODE_NORMAL);
}

void v_lora_send_frame(uint8_t u8_addr_h, uint8_t u8_addr_l, uint8_t u8_addr_c, uint8_t* au8_frame_buf, uint8_t u8_buf_size)
{
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_SET);
	v_uart2_transmit(&u8_addr_h, 1);
	v_uart2_transmit(&u8_addr_l, 1);
	v_uart2_transmit(&u8_addr_c, 1);
	v_uart2_transmit(au8_frame_buf, u8_buf_size);
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_RESET);
}




