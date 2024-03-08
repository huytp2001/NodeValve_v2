#ifndef LORA_H
#define LORA_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
	MODE_NORMAL = 0,	// can send and recieve
	MODE_WAKEUP,			// sends a preamble to waken receiver
	MODE_POWERDOWN,		// can't transmit but receive works only in wake up mode
	MODE_PROGRAM,			// for programming
} e_lora_mode_t;

#define PIN_RECOVER 50

#define SAVE_TEMPORARY 0xC2
#define SAVE_PERMANENT 0xC0

#define PB_8N1 0x00
#define PB_8O1 0x01
#define PB_8E1 0x03

#define UDR_1200 		0x00	// 1200 baud
#define UDR_2400 		0x01	// 2400 baud
#define UDR_4800 		0x02	// 4800 baud
#define UDR_9600 		0x03	// 9600 baud default
#define UDR_19200 	0x04	// 19200 baud
#define UDR_38400 	0x05	// 34800 baud
#define UDR_57600 	0x06	// 57600 baud
#define UDR_115200 	0x07	// 115200 baud

#define ADR_300 	0x00		// 300 baud
#define ADR_1200 	0x01		// 1200 baud
#define ADR_2400 	0x02		// 2400 baud
#define ADR_4800 	0x03		// 4800 baud
#define ADR_9600 	0x04		// 9600 baud
#define ADR_19200 0x05		// 19200 baud

#define FM_DISABLE 		0	
#define FM_ENABLE 		1
#define IO_OPEN_DRAIN 0	 
#define IO_PUSH_PULL  1
#define WAKEUP_250  	0x00 
#define WAKEUP_500  	0x01
#define WAKEUP_750  	0x02
#define WAKEUP_1000 	0x03
#define WAKEUP_1250 	0x04
#define WAKEUP_1500 	0x05
#define WAKEUP_1750 	0x06
#define WAKEUP_2000 	0x07
#define FEC_DISABLE		0
#define FEC_ENABLE 		1

#define OPT_TP20 0x00		// 20 db
#define OPT_TP17 0x01		// 17 db
#define OPT_TP14 0x02		// 14 db
#define OPT_TP11 0x03		// 11 db
#define OPT_TP10 0x04		// 10 db

void v_lora_set_mode(e_lora_mode_t mode);
uint8_t u8_lora_set_sped(uint8_t u8_parity_2bit, uint8_t u8_uart_bps_3bit, uint8_t u8_air_bps_3bit);
uint8_t u8_lora_set_option(bool b_fm_bit, bool b_io_driver_bit, uint8_t u8_wakeup_t_3bit, bool b_fec_bit, uint8_t u8_opt_2bit);
void v_lora_save_params(uint8_t u8_save_mode, uint8_t u8_sped, uint8_t u8_options);
void v_lora_send_frame(uint8_t u8_addr_h, uint8_t u8_addr_l, uint8_t u8_addr_c, uint8_t* au8_frame_buf, uint8_t u8_buf_size);

#ifdef __cplusplus
}
#endif
#endif
