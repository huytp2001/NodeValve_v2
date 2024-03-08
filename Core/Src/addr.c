#include "addr.h"

uint8_t u8_addr_low;
uint8_t u8_addr_high;
uint8_t u8_addr_channel;
uint8_t u8_addr_next;
uint8_t u8_addr_prev;

void v_addr_setup() 
{
	HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	u8_addr_high = 1
							 + HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) * 1
							 + HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) * 2 
							 + HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) * 4; 
							 
	u8_addr_low = 1
							 + HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) * 1
							 + HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) * 2
							 + HAL_GPIO_ReadPin(SW6_GPIO_Port, SW6_Pin) * 4; 
	if (HAL_GPIO_ReadPin(SW7_GPIO_Port, SW7_Pin) == GPIO_PIN_RESET)
	{
		u8_addr_channel = ADDR_CHAN_868MHZ;
	}
	else
	{
		u8_addr_channel = ADDR_CHAN_915MHZ;
	}
	HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_RESET);
}

bool b_is_end_node(uint8_t u8_routing)
{
	return (u8_routing >> u8_addr_low) == 0x00;
}

bool b_is_start_node(uint8_t u8_routing)
{
	return (uint8_t)(u8_routing << (9 - u8_addr_low)) == 0x00;
}

void v_addr_continuous_setup(uint8_t u8_routing)
{
	uint8_t u8_counter = 1;
	while (!((u8_routing >> (u8_addr_low + (u8_counter - 1))) & 1))
	{
		u8_counter++;
		if (u8_counter + u8_addr_low >= 8)
		{
			u8_counter = 0;
			break;
		}
	}
	u8_addr_next = u8_addr_low + u8_counter;
	u8_counter = 1;
	while (!((u8_routing >> (u8_addr_low - (u8_counter + 1))) & 1))
	{
		u8_counter++;
		if (u8_counter >= u8_addr_low)
		{
			u8_counter = 0;
			break;
		}
	}
	u8_addr_prev = u8_addr_low - u8_counter;
	if (b_is_end_node(u8_routing)) u8_addr_next = 0x00;
	if (b_is_start_node(u8_routing)) u8_addr_prev = 0x00;
}



