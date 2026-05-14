/*
 * MPU_QCM_SF.h
 *
 *  Created on: Dec 29, 2024
 *      Author: Mahmut Şare
 */
#include "stm32f1xx_hal.h"
#ifndef INC_MPU_QCM_SF_H_
#define INC_MPU_QCM_SF_H_

typedef struct  {

volatile float	accel[3];
volatile float  gyro[3];
volatile float  gyroKF[3];
volatile float  theta_accel;
volatile float  phi_accel;
volatile float  psi_magneto;//heading
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

//********************************MPU********************************
#define MPU6050_ADRESS 0x68<<1 //0110 1000 olan degeri 1101 0000 degerine kaydirarak donusturur
#define MPU_CONFIG_REG 0x1A
#define MPU_PWR_MANAGMENT_1_REG 0x6B
#define MPU_SMPLRATE_DIV_REG 0x19
#define MPU_ACCEL_CONFIG_REG 0x1C
#define MPU_GYRO_CONFIG_REG 0x1B
#define SIZE 4



//******************MPU SENSOR CONFIGURATIONS******************

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

//******************MPU SENSOR DATA REGISTERS******************

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

//******************QCM SENSOR DATA REGISTERS******************


#define QCM5883L_ADDRESS (0x0D<<1)


#define QCM_Data_Output_X_LSB 0x00
#define QCM_Data_Output_X_MSB 0x01
#define QCM_Data_Output_Y_LSB 0x02
#define QCM_Data_Output_Y_MSB 0x03
#define QCM_Data_Output_Z_LSB 0x04
#define QCM_Data_Output_Z_MSB 0x05
#define QCM_Data_Output_T_LSB 0x07
#define QCM_Data_Output_T_MSB 0x08

#define QCM5883L_CTRL_REG_1 0x09
#define QCM5883L_CTRL_REG_2 0x0A
#define QCM5883L_SET_RESET_REG 0x0B
#define QCM5883L_STATUS_REGISTER_1  0x06






#define COMPASS_CONTROL_REGISTER_1_DATA 0x99

//******************FUNCTION STRUCTURES******************

HAL_StatusTypeDef MPU6050_init_and_conf();
void MPU_GetAccelerationRawData();
void MPU_AccelerationData();
void MPU_GetGyroscopeRawData();
void MPU_GyroscopeData();
HAL_StatusTypeDef QCM_init_config();
void QCM_data ();
int invertMatrix();
void Sensor_Fusion();
#endif /* INC_MPU_QCM_SF_H_ */
