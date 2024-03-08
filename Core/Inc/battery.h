#ifndef BATTERY_H
#define BATTERY_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BAT_MILLIVOL 	3300
#define BAT_FACTOR 		2
#define BAT_LEVER			3000

uint16_t u16_bat_get_vol(void);
uint8_t u8_bat_value(uint16_t u16_bat_vol);

#ifdef __cplusplus
}
#endif
#endif


