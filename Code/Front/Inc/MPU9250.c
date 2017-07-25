/**
 * Invensense MPU-9250 library using the SPI interface
 *
 * Copyright (C) 2015 Brian Chen
 *
 * https://github.com/brianc118/MPU9250
 *
 * Open source under the MIT License. See LICENSE.txt.
 *
 * Modified for STM32F7xx.h HAL Driver
 */

//#include "stm32f7xx.h"
#include "MPU9250.h"

uint8_t MPU9250_WriteReg(MPU_SelectTypeDef *hmpu, uint8_t WriteAddr, uint8_t WriteData )
{
	uint8_t temp_buffer[2] = {WriteAddr, WriteData};
	uint8_t temp_val[2];

	MPU9250_select(hmpu);
	HAL_SPI_Transmit(&(hmpu->hspi), temp_buffer, 2, 1);
	HAL_SPI_Receive(&(hmpu->hspi), temp_val, 2, 1);
	MPU9250_deselect(hmpu);

	//HAL_Delay(50);
	return temp_val[0];
}

uint8_t  MPU9250_ReadReg(MPU_SelectTypeDef *hmpu, uint8_t WriteAddr)
{
	uint8_t temp_buffer = WriteAddr |READ_FLAG;
	uint8_t temp_val;

	MPU9250_select(hmpu);
	HAL_SPI_Transmit(&(hmpu->hspi), &temp_buffer, 1, 1);
	HAL_SPI_Receive(&(hmpu->hspi), &temp_val, 1, 1);
	MPU9250_deselect(hmpu);

	//HAL_Delay(50);
	return temp_val;
}

void MPU9250_ReadRegs(MPU_SelectTypeDef *hmpu, uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
	unsigned int  i = 0;
	uint8_t temp_buffer[Bytes];
	
	temp_buffer[0] = ReadAddr | READ_FLAG;
	for(i = 1; i < Bytes; i++) temp_buffer[i] = temp_buffer[0] + i;
	
	MPU9250_select(hmpu);
	
	for(i=0;i<Bytes;i++){
		HAL_SPI_Transmit(&hmpu->hspi, &temp_buffer[i], 1, 1);
		HAL_SPI_Receive(&hmpu->hspi, &ReadBuf[i], 1, 1);
	}
	
	MPU9250_deselect(hmpu);
	
	//HAL_Delay(5);
}


/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ 
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */

#define MPU_InitRegNum 17

int MPU9250_init(MPU_SelectTypeDef *hmpu){

	MPU9250_calibrate(hmpu, hmpu->g_bias, hmpu->a_bias);


	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
		{BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
		{0x01, MPUREG_PWR_MGMT_1},               // Clock Source
		{0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
		{hmpu->my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, hmpu->temperature bandwidth 188Hz
		{BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
		{BITS_FS_2G, MPUREG_ACCEL_CONFIG},       // +-2G
		{hmpu->my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
		{0x12, MPUREG_INT_PIN_CFG},      //
		//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
		//{0x20, MPUREG_USER_CTRL},      // Enable AUX
		{0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
		{0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz

		{AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
		//{0x09, MPUREG_I2C_SLV4_CTRL},
		//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

		{AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
		{0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
		{0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

		{AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
		#ifdef AK8963FASTMODE
		{0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
		#else
		{0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
		#endif
		{0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

	};

	for(i = 0; i < MPU_InitRegNum; i++) {
		MPU9250_WriteReg(hmpu,MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		HAL_Delay(10);  // I2C must slow down the write speed, otherwise it won't work
	}

	MPU9250_set_acc_scale(hmpu, BITS_FS_2G);
	MPU9250_set_gyro_scale(hmpu, BITS_FS_250DPS);

	MPU9250_calib_acc(hmpu);
	MPU9250_calib_mag(hmpu);  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	return 0;
}

/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */

unsigned int MPU9250_set_acc_scale(MPU_SelectTypeDef *hmpu, int scale){
    unsigned int temp_scale;
    MPU9250_WriteReg(hmpu, MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            hmpu->acc_divider=16384;
        break;
        case BITS_FS_4G:
            hmpu->acc_divider=8192;
        break;
        case BITS_FS_8G:
            hmpu->acc_divider=4096;
        break;
        case BITS_FS_16G:
            hmpu->acc_divider=2048;
        break;   
    }
    temp_scale = MPU9250_ReadReg(hmpu, MPUREG_ACCEL_CONFIG);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}



/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

unsigned int MPU9250_set_gyro_scale(MPU_SelectTypeDef *hmpu, int scale){
    unsigned int temp_scale;
    MPU9250_WriteReg(hmpu, MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   hmpu->gyro_divider = 131;  break;
        case BITS_FS_500DPS:   hmpu->gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  hmpu->gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  hmpu->gyro_divider = 16.4; break;   
    }

    temp_scale = MPU9250_ReadReg(hmpu, MPUREG_GYRO_CONFIG);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;   
    }
    return temp_scale;
}



/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 which should be 0x71
 */

unsigned int MPU9250_whoami(MPU_SelectTypeDef *hmpu){
    unsigned int response;
    response = MPU9250_ReadReg(hmpu, MPUREG_WHOAMI);
    return response;
}



/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250_read_acc(MPU_SelectTypeDef *hmpu)
{
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;
    MPU9250_ReadRegs(hmpu, MPUREG_ACCEL_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
        data = (float)bit_data;
        hmpu->accel_data[i] = data/hmpu->acc_divider - hmpu->a_bias[i];
    }
}

/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250_read_gyro(MPU_SelectTypeDef *hmpu)
{
	uint8_t response[6];
	int16_t bit_data;
	float data;
	int i;
	MPU9250_ReadRegs(hmpu, MPUREG_GYRO_XOUT_H,response,6);
	for(i = 0; i < 3; i++) {
		bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
		data = (float)bit_data;
		hmpu->gyro_data[i] = data/hmpu->gyro_divider - hmpu->g_bias[i];
	}
}


/*                                 READ hmpu->temperature
 * usage: call this function to read hmpu->temperature data. 
 * returns the value in ��C
 */

void MPU9250_read_temp(MPU_SelectTypeDef *hmpu){
	uint8_t response[2];
	int16_t bit_data;
	float data;
	MPU9250_ReadRegs(hmpu, MPUREG_TEMP_OUT_H,response,2);

	bit_data = ((int16_t)response[0]<<8)|response[1];
	data = (float)bit_data;
	hmpu->temperature = (data/340)+36.53f;
	MPU9250_deselect(hmpu);
}

/*                                 READ ACCELEROMETER CALIBRATION
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 * returns Factory Trim value
 */

void MPU9250_calib_acc(MPU_SelectTypeDef *hmpu)
{
	uint8_t response[4];
	int temp_scale;
	//READ CURRENT ACC SCALE
	temp_scale = MPU9250_ReadReg(hmpu, MPUREG_ACCEL_CONFIG);
	MPU9250_set_acc_scale(hmpu,BITS_FS_8G);
	//ENABLE SELF TEST need modify
	//temp_scale=MPU9250_WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

	MPU9250_ReadRegs(hmpu, MPUREG_SELF_TEST_X_A,response,3);
	hmpu->calib_data[0] = ((response[0]&11100000)>>3) | ((response[3]&00110000)>>4);
	hmpu->calib_data[1] = ((response[1]&11100000)>>3) | ((response[3]&00001100)>>2);
	hmpu->calib_data[2] = ((response[2]&11100000)>>3) | ((response[3]&00000011));

	MPU9250_set_acc_scale(hmpu,temp_scale);
}

uint8_t MPU9250_AK8963_whoami(MPU_SelectTypeDef *hmpu){
	uint8_t response;
	
	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

	//MPU9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
	HAL_Delay(100);
	response = MPU9250_ReadReg(hmpu, MPUREG_EXT_SENS_DATA_00);    //Read I2C 
	//ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
	//response=MPU9250_WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 

	return response;
}

void MPU9250_calib_mag(MPU_SelectTypeDef *hmpu){
	uint8_t response[3];
	float data;
	int i;
	// Choose either 14-bit or 16-bit magnetometer resolution
	//uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
	uint8_t MFS_16BITS =1; // 0.15 mG per LSB
	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	uint8_t M_8HZ = 0x02; // 8 Hz update
	//uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

	/* get the magnetometer calibration */

	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);   // Set the I2C slave    addres of AK8963 and set for read.
	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_REG, AK8963_ASAX);                 // I2C slave 0 register address from where to begin data transfer
	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_CTRL, 0x83);                       // Read 3 bytes from the magnetometer

	//MPU9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                     // Enable I2C and set bytes
	HAL_Delay(100);  
	//response[0]=MPU9250_ReadReg(MPUREG_EXT_SENS_DATA_01); //Read I2C 

	MPU9250_WriteReg(hmpu, AK8963_CNTL1, 0x00);                               // set AK8963 to Power Down
	HAL_Delay(50);                                                  // long wait between AK8963 mode changes
	MPU9250_WriteReg(hmpu, AK8963_CNTL1, 0x0F);                               // set AK8963 to FUSE ROM access
	HAL_Delay(50);                                                  // long wait between AK8963 mode changes

	MPU9250_ReadRegs(hmpu, MPUREG_EXT_SENS_DATA_00,response,3);
	//response=MPU9250_WriteReg(MPUREG_I2C_SLV0_DO, 0x00);              // Read I2C 
	for(i = 0; i < 3; i++) {
		data=response[i];
		hmpu->Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
		}
		MPU9250_WriteReg(hmpu, AK8963_CNTL1, 0x00); // set AK8963 to Power Down
		HAL_Delay(50);
		// Configure the magnetometer for continuous read and highest resolution.
		// Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
		// register, and enable continuous mode data acquisition (bits [3:0]),
		// 0010 for 8 Hz and 0110 for 100 Hz sample rates.   
		MPU9250_WriteReg(hmpu, AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ);            // Set magnetometer data resolution and sample ODR
		HAL_Delay(50);
	}

void MPU9250_read_mag(MPU_SelectTypeDef *hmpu){
	uint8_t response[7];
	float data;
	int i;

	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_CTRL, 0x87);                      // Read 6 bytes from the magnetometer

	// delayMicroseconds(1000);
	MPU9250_ReadRegs(hmpu, MPUREG_EXT_SENS_DATA_00,response,7);
	// must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
	for(i = 0; i < 3; i++) {
		hmpu->mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
		data = (float)hmpu->mag_data_raw[i];
		hmpu->mag_data[i] = data*hmpu->Magnetometer_ASA[i];
	}
}

uint8_t MPU9250_get_CNTL1(MPU_SelectTypeDef *hmpu){
	MPU9250_WriteReg(hmpu, MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_REG, AK8963_CNTL1);              // I2C slave 0 register address from where to begin data transfer
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

	// delayMicroseconds(1000);
	return MPU9250_ReadReg(hmpu, MPUREG_EXT_SENS_DATA_00);    //Read I2C 
}

void MPU9250_read_all(MPU_SelectTypeDef *hmpu){
	uint8_t response[21];
	int16_t bit_data;
	float data;
	int i;

	// Send I2C command at first
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
	MPU9250_WriteReg(hmpu,MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
	// must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

	MPU9250_ReadRegs(hmpu,MPUREG_ACCEL_XOUT_H,response,21);
	// Get accelerometer value
	for(i = 0; i < 3; i++) {
		bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
		data = (float)bit_data;
		hmpu->accel_data[i] = data/hmpu->acc_divider - hmpu->a_bias[i];
	}
	// Get hmpu->temperature
	bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
	data = (float)bit_data;
	hmpu->temperature = ((data-21)/333.87f)+21;
	// Get gyroscope value
	for(i=4; i < 7; i++) {
		bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
		data = (float)bit_data;
		hmpu->gyro_data[i-4] = data/hmpu->gyro_divider - hmpu->g_bias[i-4];
	}
	// Get Magnetometer value
	for(i=7; i < 10; i++) {
		hmpu->mag_data_raw[i-7] = ((int16_t)response[i*2+1]<<8) | response[i*2];
		data = (float)hmpu->mag_data_raw[i-7];
		hmpu->mag_data[i-7] = data * hmpu->Magnetometer_ASA[i-7];
	}
}

void MPU9250_calibrate(MPU_SelectTypeDef *hmpu, float *dest1, float *dest2){  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    MPU9250_WriteReg(hmpu, MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);
   
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    MPU9250_WriteReg(hmpu, MPUREG_PWR_MGMT_1, 0x01);  
    MPU9250_WriteReg(hmpu, MPUREG_PWR_MGMT_2, 0x00);
    HAL_Delay(200);                                    

    // Configure device for bias calculation
    MPU9250_WriteReg(hmpu, MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    MPU9250_WriteReg(hmpu, MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    MPU9250_WriteReg(hmpu, MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    MPU9250_WriteReg(hmpu, MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    MPU9250_WriteReg(hmpu, MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    MPU9250_WriteReg(hmpu, MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    HAL_Delay(15);
  
    // Configure MPU6050 gyro and accelerometer for bias calculation
    MPU9250_WriteReg(hmpu, MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    MPU9250_WriteReg(hmpu, MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    MPU9250_WriteReg(hmpu, MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    MPU9250_WriteReg(hmpu, MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    MPU9250_WriteReg(hmpu, MPUREG_USER_CTRL, 0x40);   // Enable FIFO  
    MPU9250_WriteReg(hmpu, MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    MPU9250_WriteReg(hmpu, MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    MPU9250_ReadRegs(hmpu, MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
    
    for (ii = 0; ii < packet_count; ii++) {
	int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	MPU9250_ReadRegs(hmpu, MPUREG_FIFO_R_W, data, 12); // read data for averaging
	accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
	accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
	accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
	gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
	gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
	gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

	accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	accel_bias[1] += (int32_t) accel_temp[1];
	accel_bias[2] += (int32_t) accel_temp[2];
	gyro_bias[0]  += (int32_t) gyro_temp[0];
	gyro_bias[1]  += (int32_t) gyro_temp[1];
	gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
    // Push gyro biases to hardware registers
    MPU9250_WriteReg(hmpu, MPUREG_XG_OFFS_USRH, data[0]);
    MPU9250_WriteReg(hmpu, MPUREG_XG_OFFS_USRL, data[1]);
    MPU9250_WriteReg(hmpu, MPUREG_YG_OFFS_USRH, data[2]);
    MPU9250_WriteReg(hmpu, MPUREG_YG_OFFS_USRL, data[3]);
    MPU9250_WriteReg(hmpu, MPUREG_ZG_OFFS_USRH, data[4]);
    MPU9250_WriteReg(hmpu, MPUREG_ZG_OFFS_USRL, data[5]);
  
    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for hmpu->temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    MPU9250_ReadRegs(hmpu, MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    MPU9250_ReadRegs(hmpu, MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    MPU9250_ReadRegs(hmpu, MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    
    uint32_t mask = 1uL; // Define mask for hmpu->temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If hmpu->temperature compensation bit is set, record that fact in mask_bit
    }
    
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
  
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve hmpu->temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve hmpu->temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve hmpu->temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the hmpu->temperature correction bit properly?
// Push accelerometer biases to hardware registers
    MPU9250_WriteReg(hmpu, MPUREG_XA_OFFSET_H, data[0]);
    MPU9250_WriteReg(hmpu, MPUREG_XA_OFFSET_L, data[1]);
    MPU9250_WriteReg(hmpu, MPUREG_YA_OFFSET_H, data[2]);
    MPU9250_WriteReg(hmpu, MPUREG_YA_OFFSET_L, data[3]);
    MPU9250_WriteReg(hmpu, MPUREG_ZA_OFFSET_H, data[4]);
    MPU9250_WriteReg(hmpu, MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void MPU9250_select(MPU_SelectTypeDef *hmpu) {
	//Set CS low to start transmission (interrupts conversion)
	HAL_GPIO_WritePin(hmpu->GPIOx, hmpu->GPIO_PIN, GPIO_PIN_RESET);
}
void MPU9250_deselect(MPU_SelectTypeDef *hmpu) {
	//Set CS high to stop transmission (restarts conversion)
	HAL_GPIO_WritePin(hmpu->GPIOx, hmpu->GPIO_PIN, GPIO_PIN_SET);
}




void MPU9250_SelfTest(MPU_SelectTypeDef *hmpu, float *destination)
{

	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;
	int ii;

	MPU9250_WriteReg(hmpu, MPUREG_SMPLRT_DIV, 0x00);	// Set gyro sample rate to 1 kHz
	MPU9250_WriteReg(hmpu, MPUREG_CONFIG, 0x02);		// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	MPU9250_WriteReg(hmpu, MPUREG_GYRO_CONFIG, 1<<FS);	// Set full scale range for the gyro to 250 dps
	MPU9250_WriteReg(hmpu,  MPUREG_ACCEL_CONFIG_2, 0x02);	// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	MPU9250_WriteReg(hmpu,  MPUREG_ACCEL_CONFIG, 1<<FS);	// Set full scale range for the accelerometer to 2 g
	
	// Get average current values of gyro and acclerometer

	for (ii = 0; ii < 200; ii++)
	{
	// Read the six raw data registers into data array

		MPU9250_ReadRegs(hmpu,  MPUREG_ACCEL_XOUT_H, rawData, 6);

		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;	// Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		
		MPU9250_ReadRegs(hmpu,  MPUREG_GYRO_XOUT_H, rawData, 6);			// Read the six raw data registers sequentially into data array

		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;	// Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	  }



  // Get average of 200 values and store as average current readings

	for (int ii =0; ii < 3; ii++)
	{
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test

	MPU9250_WriteReg(hmpu, MPUREG_ACCEL_CONFIG, 0xE0);		// Enable self test on all three axes and set accelerometer range to +/- 2 g
	MPU9250_WriteReg(hmpu, MPUREG_GYRO_CONFIG,  0xE0);		// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	
	HAL_Delay(25);  // Delay a while to let the device stabilize
	// Get average self-test values of gyro and acclerometer

	for (int ii = 0; ii < 200; ii++)

	{
		MPU9250_ReadRegs(hmpu, MPUREG_ACCEL_XOUT_H, rawData, 6);		// Read the six raw data registers into data array

		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; 			// Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		MPU9250_ReadRegs(hmpu, MPUREG_GYRO_XOUT_H, rawData, 6);			// Read the six raw data registers sequentially into data array

		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;			// Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++)		// Get average of 200 values and store as average self-test readings
	{
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	MPU9250_WriteReg(hmpu, MPUREG_ACCEL_CONFIG, 0x00);		// Configure the gyro and accelerometer for normal operation
	MPU9250_WriteReg(hmpu, MPUREG_GYRO_CONFIG,  0x00);

	HAL_Delay(25);  // Delay a while to let the device stabilize



	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg

	selfTest[0] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_X_A);	// X-axis accel self-test results
	selfTest[1] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_Y_A);	// Y-axis accel self-test results
	selfTest[2] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_Z_A);	// Z-axis accel self-test results

	selfTest[3] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_X_G);	// X-axis gyro self-test results
	selfTest[4] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_Y_G);	// Y-axis gyro self-test results
	selfTest[5] = MPU9250_ReadReg(hmpu, MPUREG_SELF_TEST_Z_G);	// Z-axis gyro self-test results



	// Retrieve factory self-test value from self-test code reads

	

	factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));	// FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));	// FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));	// FT[Za] factory trim calculation

	factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));	// FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));	// FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));	// FT[Zg] factory trim calculation



	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim

	// of the Self-Test Response

	// To get percent, must multiply by 100

	for (int i = 0; i < 3; i++)
	{
		// Report percent differences
		destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;

		// Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.;
	}

}
