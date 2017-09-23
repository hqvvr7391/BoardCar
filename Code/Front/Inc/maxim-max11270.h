#ifndef _MAXIM_MAX11270
#define _MAXIM_MAX11270

#include "stm32f7xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

typedef struct 
{
	SPI_HandleTypeDef	hspi;

	GPIO_TypeDef		*CSB_GPIOx;
	uint16_t 			CSB_GPIO_PIN;
	
	GPIO_TypeDef		*RSTB_GPIOx;
	uint16_t			RSTB_GPIO_PIN;
	
	GPIO_TypeDef		*SYNC_GPIOx;
	uint16_t			SYNC_GPIO_PIN;
	
	GPIO_TypeDef		*RDYB_GPIOx;
	uint16_t			RDYB_GPIO_PIN;
	
	
	uint8_t	RATE;
	uint8_t	GAIN;
	
	uint8_t	Conv_Data[3];
	uint32_t	Conv_TData;
	
	float Data[3];
	float temperature;

} MAX_SelectTypeDef;



// MAX11210 8-bit Command unsigned char

// ------------------------------------------------------------------------

// name     B7         B6         B5     B4     B3     B2     B1     B0 

// ------------------------------------------------------------------------

// COMMAND  START = 1  MODE = 0   CAL   IMPD   RATE3  RATE2  RATE1  RATE0

// COMMAND  START = 1  MODE = 1   0      RS3    RS2    RS1    RS0    R/!W

#define START 0x80

#define MODE  0x40

#define CAL  0x20

#define IMPD  0x08

#define READ  0x01

#define MAX11270_RATE50 0x00
#define MAX11270_RATE62_5 0x01
#define MAX11270_RATE100 0x02
#define MAX11270_RATE125 0x03
#define MAX11270_RATE200 0x04
#define MAX11270_RATE250 0x05
#define MAX11270_RATE400 0x06
#define MAX11270_RATE500 0x07
#define MAX11270_RATE800 0x08
#define MAX11270_RATE1000 0x09
#define MAX11270_RATE1600 0x0A
#define MAX11270_RATE2000 0x0B
#define MAX11270_RATE3200 0x0C
#define MAX11270_RATE4000 0x0D
#define MAX11270_RATE6400 0x0E
#define MAX11270_RATE12800 0x0F

// MAX11210 Status & control registers

// ------------------------------------------------------------------------

//     name     B7     B6     B5     B4     B3     B2     B1     B0

// ------------------------------------------------------------------------

// 0x0 STAT   INRESET  ERROR  —     —   PDSTAT1 PDSTAT0 RDERR AOR 
//			   RATE3  RATE2 RATE1 RATE0  SYSGOR  DOR    MSTAT   RDY 

// 0x1 CTRL1    EXTCK SYNCMODE PD1 PD0   U/~B   FORMAT  SCYCLE  CONTSC 

// 0x2 CTRL2    DGAIN1 DGAIN0 BUFEN LPMODE PGAEN  PGAG2  PGAG1   PGAG0 

// 0x3 CTRL3     —     —   ENMSYNC MODBITS DATA32  —     —       —


#define STAT1  0x00

#define CTRL1  0x01

#define CTRL2  0x02

#define CTRL3  0x03

#define DATA   0x06






// CTRL1

#define EXTCK  0x80
#define SYNC   0x40
#define U 0x08
#define FORMAT 0x04
#define SCYCLE 0x02
#define CONTSC 0x01



#define MAX_11270_NORMAL_POWER = 0x00;
#define MAX_11270_SLEEP_POWER = 0x10;
#define MAX_11270_STANDBY_POWER = 0x20;
#define MAX_11270_POR_STATE = 0x30;




// CTRL2

#define PGAEN 0x08
#define LPMODE 0x10
#define BUFEN 0x20


#define MAX11270_GAIN1 0x00
#define MAX11270_GAIN2 0x01
#define MAX11270_GAIN4 0x02
#define MAX11270_GAIN8 0x03
#define MAX11270_GAIN16 0x04
#define MAX11270_GAIN32 0x05
#define MAX11270_GAIN64 0x06
#define MAX11270_GAIN128 0x07
#define MAX11270_DGAIN1 0x00
#define MAX11270_DGAIN2 0x40
#define MAX11270_DGAIN4 0x80
#define MAX11270_DGAIN8 0xC0

#endif


void MAX11270_ConvCmd(MAX_SelectTypeDef *hmax);
void MAX11270_WriteReg8(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t conf);
void MAX11270_ReadReg8(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t *buffer);
void MAX11270_ReadReg16(MAX_SelectTypeDef *hmax, uint8_t reg, uint8_t *buffer);
void MAX11270_DataRead(MAX_SelectTypeDef *hmax);
//void Max11270_end_1(uint8_t reg, uint8_t conf);
void MAX11270_Init(MAX_SelectTypeDef *hmax);
void MAX11270_Select(MAX_SelectTypeDef *hmax);
void MAX11270_Deselect(MAX_SelectTypeDef *hmax);
void MAX11270_RSTB_Set(MAX_SelectTypeDef *hmax);
void MAX11270_RSTB_Reset(MAX_SelectTypeDef *hmax);
void MAX11270_SYNC_Set(MAX_SelectTypeDef *hmax);
void MAX11270_SYNC_Reset(MAX_SelectTypeDef *hmax);