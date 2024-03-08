#include "flash.h"

uint32_t u32_flash_write(uint8_t *data, uint16_t size_byte)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = 0x0801FC00;
	EraseInitStruct.NbPages     = 1;
	uint32_t PAGEError = 0;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError))
	{
		return HAL_FLASH_GetError();
	}
	for (uint8_t i = 0; i < size_byte; i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0801FC00 + i, data[i]) != HAL_OK)
		{
			return HAL_FLASH_GetError();
		}
	}
	HAL_FLASH_Lock();
	return 0;
}

uint8_t u8_flash_read(uint8_t index)
{
	uint8_t data = *(__IO uint8_t *)(0x0801FC00 + index);
	return data;
}

