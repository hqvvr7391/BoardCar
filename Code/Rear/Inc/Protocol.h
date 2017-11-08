#ifndef Protocol
#define Protocol

#include <stdint.h>

#include "stm32f7xx_hal.h"
#include "main.h"


#define Protocol_StartB	0x80
#define Protocol_EndB	0x05


void Data_Transmit(uint8_t Device, uint32_t data,uint16_t Timeout);

#endif
