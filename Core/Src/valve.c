#include "valve.h"
#include "addr.h"
#include "flash.h"

static uint8_t u8_valve_get_new_state(uint32_t u32_state_val);
static uint8_t u8_valve_get_state(void);
static void v_valve_control_port(uint8_t u8_port, uint8_t u8_state);

bool b_valve_check(uint8_t u8_en_state_val)
{	
	if ((u8_en_state_val>>(u8_addr_low-1))&0x01)
	{
		return true;
	}
	return false;
}

static uint8_t u8_valve_get_new_state(uint32_t u32_state_val)
{
	return u32_state_val >> 4*(u8_addr_low - 1);
}

static uint8_t u8_valve_get_state(void)
{
	uint8_t u8_curr_state = 0x00;
	if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)) u8_curr_state |= 0x01;
	if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)) u8_curr_state |= 0x02;
	if (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)) u8_curr_state |= 0x04;
	if (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin)) u8_curr_state |= 0x08;
	return u8_curr_state; 
}

void v_valve_control_u32(uint32_t u32_new_state_val)
{
	uint8_t u8_new_state_val = u8_valve_get_new_state(u32_new_state_val);
	uint8_t u8_old_state_val = u8_valve_get_state();
	u8_old_state_val ^= u8_new_state_val;
	for (uint8_t i = 0; i < 4; i++) 
	{
		if ((u8_old_state_val >> i) & 0x01) v_valve_control_port(i+1, (u8_new_state_val >> i) & 0x01);
	}
	u32_flash_write((uint8_t*)&u8_new_state_val, 1);
}	

static void v_valve_control_port(uint8_t u8_port, uint8_t u8_state)
{
	if (u8_state) // turn on
	{
		switch (u8_port)
		{
			case 1:
		{
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		}
		break;
		case 2:
		{
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
		break;
		case 3:
		{
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		}
		break;
		case 4:
		{
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}
		break;
		default:
		break;
		}
	}
	else // turn off
	{
		switch (u8_port)
		{
			case 1:
		{
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}
		break;
		case 2:
		{
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		break;
		case 3:
		{
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		}
		break;
		case 4:
		{
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		}
		break;
		default:
		break;
		}
	}
}









