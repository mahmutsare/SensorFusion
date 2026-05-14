/*
 * sensor_fusion_lib.h
 *
 *  Created on: Dec 29, 2024
 *      Author: Mahmut Şare
 */

#ifndef INC_SENSOR_FUSION_LIB_H_
#define INC_SENSOR_FUSION_LIB_H_
#endif /* INC_SENSOR_FUSION_LIB_H_ */



#define MPU6050_ADRESS 0x68<<1 //0110 1000 olan degeri 1101 0000 degerine kaydirarak donusturur
#define MPU_CONFIG_REG 0x1A
#define MPU_PWR_MANAGMENT_1_REG 0x6B
#define MPU_SMPLRATE_DIV_REG 0x19
#define MPU_ACCEL_CONFIG_REG 0x1C
#define MPU_GYRO_CONFIG_REG 0x1B
#define SIZE 4





typedef struct  {

volatile float	accel[3];
volatile float  gyro[3];
volatile float  gyroKF[3];
volatile float  theta_accel;
volatile float  phi_accel;
volatile float  THETA;
volatile float  PHI;
volatile float  PSI;
} sensor_t ;

typedef struct {
	volatile float Q[4][4];
    volatile float R[4][4];
    volatile float P[4][4];
    volatile float P_k_upminus[4][4];
	const    float time_interval;
	volatile float K_gain[4][4];
	volatile float q_est[4];
	volatile float q_pred[4];
	volatile float q_accel[4];
} SF_KF_t;


typedef struct {
	volatile float Ppredgy[3];
	volatile float Kgain_gy[3];
	volatile float Pgy[3];
	const float Qgy[3];
	const float Rgy[3];
} kucuk_kalman;


//******************SENSOR CONFIGURATIONS******************

//POWER MANAGMENT MEM DATA
/*
 *
 */
#define PWR_MANAGMENT_DT 0x00


//CONFIG MEM DATA
#define CONFIG_DT 0x00


//GYRO CONFIG MEM DATA
/* +- 250  deg/s      0x00
 * +- 500  deg/s	  0x08
 * +- 1000 deg/s	  0x10
 * +- 2000 deg/s	  0x18
 * */
#define GYRO_CONFIG_DT 0x00



//ACCELERATION CONFIG MEM DATA
/* +- 2g     0x00
 * +- 4g	 0x08
 * +- 8g	 0x10
 * +- 16g	 0x18
 * */
#define ACCEL_CONFIG_DT 0x00

//SAMPLE RATE DIVIDER MEM DATA
#define SMDV 0x00

//******************SENSOR DATA REGISTERS******************

#define TEMP_MEASUREMENT_MSB 0x41
#define TEMP_MEASUREMENT_LSB 0x42

//gyroX measurements
#define GYROX_MSB 0x43
#define GYROX_LSB 0x44
//gyroY measurements
#define GYROY_MSB 0x45
#define GYROY_LSB 0x46
//gyroZ measurements
#define GYROZ_MSB 0x47
#define GYROZ_LSB 0x48

//accelX measurements
#define ACCELX_MSB 0x3B
#define ACCELX_LSB 0x3C

//accelY measurements
#define ACCELY_MSB 0x3D
#define ACCELY_LSB 0x3E
//accelZ measurements
#define ACCELZ_MSB 0x3F
#define ACCELZ_LSB 0x40


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


