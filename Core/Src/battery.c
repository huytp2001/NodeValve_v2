#include "battery.h"

uint16_t u16_bat_get_vol(void)
{
	uint16_t u16_real_value = 0;
	uint16_t u16_raw_value = 0;
	u16_raw_value = u16_adc_get_value();
	u16_real_value = u16_raw_value * ((float)BAT_MILLIVOL / 4096) * BAT_FACTOR + 200;
	return u16_real_value;
}

uint8_t u8_bat_value(uint16_t u16_bat_vol)
{
	if (u16_bat_vol >= BAT_LEVER) return 0x01; else return 0x00;
}
