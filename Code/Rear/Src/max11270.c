#include "maxim-max11270.h"


////////////>>>>>>>>>>SPI1<<<<<<<<<<<<<<<///////////////
void Max11270_CovCmd_1(uint8_t rate) 
{
	uint8_t mesg = (rate & 0x0F)|START;
	
	CSB_LOW_1;
	HAL_SPI_Transmit(&hspi1,&mesg,1,1);
	CSB_HIGH_1;
	
	HAL_Delay(10);
}

void Max11270_writeReg8_1(uint8_t reg, uint8_t conf) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE;
	
	CSB_LOW_1;
	HAL_SPI_Transmit(&hspi1, &mesg, 1, 0);
	HAL_SPI_Transmit(&hspi1, &conf, 1, 0);
	CSB_HIGH_1;

}

void Max11270_readReg8_1(uint8_t reg, uint8_t *buffer) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE | READ;
	
	CSB_LOW_1;
	HAL_SPI_Transmit(&hspi1, &mesg, 1, 1);
	HAL_SPI_Receive(&hspi1, buffer, 1, 1);
	CSB_HIGH_1;

}


void Max11270_DataRead_1(uint8_t * buffer) 
{
	uint8_t mesg = 0xCD;

	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
	{
		CSB_LOW_1;
		HAL_SPI_Transmit(&hspi1, &mesg, 1, 1);
		HAL_SPI_Receive(&hspi1, buffer, 3, 1);
		CSB_HIGH_1;
	}

	
		
}






////////////>>>>>>>>>>>>>SPI2<<<<<<<<<<<<</////////////////////

void Max11270_CovCmd_2(uint8_t rate) 
{
	uint8_t mesg = (rate & 0x0F)|START;
	
	CSB_LOW_2;
	HAL_SPI_Transmit(&hspi2,&mesg,1,0);
	CSB_HIGH_2;
	
	HAL_Delay(10);
}

void Max11270_writeReg8_2(uint8_t reg, uint8_t conf) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE;
	
	CSB_LOW_2;
	HAL_SPI_Transmit(&hspi2, &mesg, 1, 0);
	HAL_SPI_Transmit(&hspi2, &conf, 1, 0);
	CSB_HIGH_2;

}

void Max11270_readReg8_2(uint8_t reg, uint8_t *buffer) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE | READ;
	
	CSB_LOW_2;
	HAL_SPI_Transmit(&hspi2, &mesg, 1, 1);
	HAL_SPI_Receive(&hspi2, buffer, 1, 1);
	CSB_HIGH_2;

}


void Max11270_DataRead_2(uint8_t * buffer) 
{
	uint8_t mesg = 0xCD;

	
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == 0)
	{
		CSB_LOW_2;
		HAL_SPI_Transmit(&hspi2, &mesg, 1, 0);
		HAL_SPI_Receive(&hspi2, buffer, 3, 1);
		CSB_HIGH_2;
	}
	
		
}



/*void Max11270_end(uint8_t reg, uint8_t conf)
{
	
	Max11270_writeReg8(reg, conf);
		
	HAL_SPI_DeInit(&hspi2);

}*/