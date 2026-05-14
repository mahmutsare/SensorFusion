/*
 * sensor_fusion_lib.h
 *
 *  Created on: Dec 29, 2024
 *      Author: Mahmut Şare
 */

#ifndef INC_SENSOR_FUSION_LIB_H_
#define INC_SENSOR_FUSION_LIB_H_
#endif /* INC_SENSOR_FUSION_LIB_H_ */











typedef struct {
	volatile float Ppredgy[3];
	volatile float Kgain_gy[3];
	volatile float Pgy[3];
	const float Qgy[3];
	const float Rgy[3];
} kucuk_kalman;




// Kalman Filter
void Sensor_Fusion();
int MPU6050_init_and_conf();
//***************************DATA PROCESS***************************
void MPU_GetAccelerationRawData();
void MPU_AccelerationData();
void MPU_GetGyroscopeRawData();
void MPU_GyroscopeData();
//************************Calibration*****************************
void MPU6050_AccelCal();
void MPU6050_GyroCal();
//*******************************************************MATH FUNCTIONS*******************************************************
int invertMatrix();


