/*#include "maxim-max11270.h"


void MAX11270_ConvCmd(MAX_SelectTypeDef *hmax) 
{
	uint8_t mesg = (hmax->RATE & 0x0F) | START;
	
	MAX11270_Select(hmax);
	HAL_SPI_Transmit(&hmax->hspi,&mesg,1,1);
	MAX11270_Deselect(hmax);
	
	//HAL_Delay(10);
}


void MAX11270_WriteReg8(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t conf) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE;
	
	MAX11270_Select(hmax);
	HAL_SPI_Transmit(&hmax->hspi, &mesg, 1, 10);
	HAL_SPI_Transmit(&hmax->hspi, &conf, 1, 10);
	MAX11270_Deselect(hmax);
	
	
}

void MAX11270_ReadReg8(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t *buffer) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE | READ;
	
	MAX11270_Select(hmax);
	HAL_SPI_Transmit(&hmax->hspi, &mesg, 1, 10);
	HAL_SPI_Receive(&hmax->hspi, buffer, 1, 10);
	MAX11270_Deselect(hmax);

}

void MAX11270_ReadReg16(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t *buffer) 
{
	uint8_t mesg = ((reg & 0x1F) << 1) | START | MODE | READ;
	
	MAX11270_Select(hmax);
	HAL_SPI_Transmit(&hmax->hspi, &mesg, 1, 10);
	HAL_SPI_Receive(&hmax->hspi, buffer, 2, 10);
	MAX11270_Deselect(hmax);

}

void MAX11270_DataRead(MAX_SelectTypeDef *hmax) 
{
	uint8_t mesg = 0xCD;

	//while(HAL_GPIO_ReadPin(hmax->RDYB_GPIOx, hmax->RDYB_GPIO_PIN) != RESET);
	
	MAX11270_Select(hmax);
	HAL_SPI_Transmit(&hmax->hspi, &mesg, 1, 10);
	HAL_SPI_Receive(&hmax->hspi, hmax->Conv_Data, 3, 10);
	MAX11270_Deselect(hmax);
	
	hmax->Conv_TData = hmax->Conv_Data[0] << 16 |  hmax->Conv_Data[1] << 8 | hmax->Conv_Data[2];
}

void MAX11270_Init(MAX_SelectTypeDef *hmax)
{
	uint8_t Conf;
	
	MAX11270_RSTB_Set(hmax);
	MAX11270_SYNC_Reset(hmax);
	
	Conf = FORMAT | U |SCYCLE | CONTSC;
	
	MAX11270_WriteReg8(hmax, CTRL1, Conf);
	
	HAL_Delay(10);
	
	Conf =PGAEN | hmax->GAIN;
	
	MAX11270_WriteReg8(hmax, CTRL2, Conf);
	HAL_Delay(10);
}


void MAX11270_Select(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->CSB_GPIOx,hmax->CSB_GPIO_PIN,GPIO_PIN_RESET);
}

void MAX11270_Deselect(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->CSB_GPIOx,hmax->CSB_GPIO_PIN,GPIO_PIN_SET);
}

void MAX11270_RSTB_Set(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->RSTB_GPIOx,hmax->RSTB_GPIO_PIN,GPIO_PIN_SET);
}

void MAX11270_RSTB_Reset(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->CSB_GPIOx,hmax->CSB_GPIO_PIN,GPIO_PIN_RESET);
}

void MAX11270_SYNC_Set(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->SYNC_GPIOx,hmax->SYNC_GPIO_PIN,GPIO_PIN_SET);
}

void MAX11270_SYNC_Reset(MAX_SelectTypeDef *hmax)
{
	HAL_GPIO_WritePin(hmax->SYNC_GPIOx,hmax->SYNC_GPIO_PIN,GPIO_PIN_RESET);
}
*/
/*void Max11270_end(uint8_t reg, uint8_t conf)
{
	
	Max11270_writeReg8(reg, conf);
		
	HAL_SPI_DeInit(&hspi2);

}*/

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
