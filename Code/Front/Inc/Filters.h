// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"

// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)

// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute

// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.

// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms

// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
#ifndef Filters_H

#define Filters_H

#include <math.h>

#include "stm32f7xx.h"
#include "Queue.h"

#define PI	3.14159265358979323846f

#define delta_t	0.001f
#define gyroMeasError 	PI * (60.0f / 180.0f)
#define gyroMeasDrift	GyroMeasDrift = PI * (1.0f / 180.0f)
#define beta	sqrt(3.0f / 4.0f) * gyroMeasError
#define zeta	sqrt(3.0f / 4.0f) * gyroMeasError


#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral

#define Ki 0.0f

float MoveAverage(Queue_TypeDef *q);
float Complemetary(float C_alpha, float a, float b);
float *MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
float *MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	
#endif
