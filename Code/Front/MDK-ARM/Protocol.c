#include "main.h"
#include "Protocol.h"

extern UART_HandleTypeDef huart6;
extern uint32_t Strain_Gauge[4];

void Data_Transmit(uint8_t Device, uint32_t data,uint16_t Timeout) {
	
	uint8_t data_Array[9];
	uint8_t i;
	
	data_Array[0] = Protocol_StartB;
	data_Array[1] = Device;
	
	for(i=2; i<5; i++)	{
		data_Array[i] = (uint8_t) ((data >> ((4-i)*8) ) & 0xFF);
		data_Array[i+3] =data_Array[i];
		
	}
	data_Array[8] = Protocol_EndB;
	
	HAL_UART_Transmit(&huart6, data_Array, 9, Timeout);
	

}