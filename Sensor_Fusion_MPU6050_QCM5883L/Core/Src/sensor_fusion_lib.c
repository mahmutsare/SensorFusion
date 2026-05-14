/*
 * sensor_fusion_lib.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Mahmut Şare
 */


/*







//***************************DATA PROCESS***************************








//************************Calibration*****************************

void MPU6050_AccelCal(I2C_HandleTypeDef *i2c, int32_t accel_cal[]) {
	int16_t accel_raw[3];
	for (int i =1;  i<2000; i++ ){
			MPU_GetAccelerationRawData(i2c, &accel_raw);
			accel_cal[0]+=accel_raw[0];
			accel_cal[1]+=accel_raw[1];
			accel_cal[2]+=accel_raw[2];}
		accel_cal[0]/=2000;
		accel_cal[1]/=2000;
		accel_cal[2]=accel_cal[2]/2000;// kalibrasyonda sensoru z ekseninde dik tuttugumuzda 1g'nin karsiligi 16384'tur o nedenle gurultu
		// olarak sayilmamasi icin cikariyoruz
}

void MPU6050_GyroCal(I2C_HandleTypeDef *i2c, int32_t gyro_cal[]) {
	int16_t gyro_raw[3];
		for (int i =0; i<20000; i++ ){
			MPU_GetGyroscopeRawData(i2c, &gyro_raw);
			gyro_cal[0]+=gyro_raw[0];
			gyro_cal[1]+=gyro_raw[1];
			gyro_cal[2]+=gyro_raw[2];
		}
		gyro_cal[0]/=20000;
		gyro_cal[1]/=20000;
		gyro_cal[2]/=20000;
}

//*******************************************************MATH FUNCTIONS*******************************************************


*/
