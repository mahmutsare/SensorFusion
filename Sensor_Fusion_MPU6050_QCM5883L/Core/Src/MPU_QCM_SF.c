/*
 * MPU_QCM_SF.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Mahmut Şare
 */

#include "MPU_QCM_SF.h"
#include "main.h"
//#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>





//******************MPU FUNC******************


HAL_StatusTypeDef MPU6050_init_and_conf(I2C_HandleTypeDef *i2c) {//sensorun calisma durumunu kontrol eder ve konfigurasyonlari yapar
	  HAL_StatusTypeDef ret=    HAL_I2C_IsDeviceReady(i2c, MPU6050_ADRESS, 10, 1);
	  // power management register'ina data verisini yazdiriyoruz Kaynak: register map page: 41
	  HAL_StatusTypeDef ret_pwr=    HAL_I2C_Mem_Write(i2c, MPU6050_ADRESS, MPU_PWR_MANAGMENT_1_REG, 1, PWR_MANAGMENT_DT, 1, 1);
	  // config register'ina data verisini yazdiriyoruz Kaynak: register map page: 13
	  HAL_StatusTypeDef ret_conf=   HAL_I2C_Mem_Write(i2c, MPU6050_ADRESS, MPU_CONFIG_REG         , 1, CONFIG_DT       , 1, 1);
	  //SAMPLE DIVIDER register'ına veriyi yazdiriyoruz Kaynak: register map page:11-12

	  HAL_StatusTypeDef ret_dv=     HAL_I2C_Mem_Write(i2c, MPU6050_ADRESS, MPU_SMPLRATE_DIV_REG   , 1, SMDV            , 1, 1);
	  //gyro config  Kaynak: register map page:14
	  HAL_StatusTypeDef ret_gyconf= HAL_I2C_Mem_Write(i2c, MPU6050_ADRESS, MPU_GYRO_CONFIG_REG    , 1, GYRO_CONFIG_DT  , 1, 1);
	  //accel config Kaynak: register map page:15
	  HAL_StatusTypeDef ret_acconf= HAL_I2C_Mem_Write(i2c, MPU6050_ADRESS, MPU_ACCEL_CONFIG_REG   , 1, ACCEL_CONFIG_DT , 1, 1);

	  if (ret== 0 && ret_conf== 0 && ret_dv== 0 && ret_gyconf== 0 && ret_acconf== 0 && ret_pwr== 0){//0 olması islemin gerceklestirildigi anlamina gelir
	  	return HAL_OK;
	    }
	  else {
		  return HAL_ERROR;
	  }
}

void MPU_GetAccelerationRawData(I2C_HandleTypeDef *i2c, int16_t accel_raw[] ) {
	uint8_t read_accel[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADRESS, ACCELX_MSB, 1, &read_accel, 6, HAL_MAX_DELAY);
	accel_raw[0]=((int16_t)read_accel[0]<<8)  +  read_accel[1];
	accel_raw[1]=((int16_t)read_accel[2]<<8)  +  read_accel[3];
	accel_raw[2]=((int16_t)read_accel[4]<<8)  +  read_accel[5];

	  }
void MPU_AccelerationData(I2C_HandleTypeDef *i2c,  sensor_t *veriler ) {
	int16_t accel_raw[3];
	MPU_GetAccelerationRawData(i2c, accel_raw );
    veriler->accel[0]=(float)-(accel_raw[0]- 1018)*0.0000610352;//   2^(16)/2/2g=16384 LSB/g  (+- 2g okudugumuz icin)
	veriler->accel[1]=(float)-(accel_raw[1]+   50)*0.0000610352;//  1/16384 LSB/g = 0.0000610352 g/LSB
	veriler->accel[2]=(float)-(accel_raw[2]- 2010)*0.0000610352;
}

void MPU_GetGyroscopeRawData(I2C_HandleTypeDef *i2c, int16_t gyro_raw[] ){
	uint8_t read_gyro[6];

	HAL_I2C_Mem_Read(i2c, MPU6050_ADRESS, GYROX_MSB, 1, &read_gyro, 6, HAL_MAX_DELAY);
	gyro_raw[0]=((int16_t)read_gyro[0]<<8)  +  read_gyro[1];
	gyro_raw[1]=((int16_t)read_gyro[2]<<8)  +  read_gyro[3];
	gyro_raw[2]=((int16_t)read_gyro[4]<<8)  +  read_gyro[5];
}

void MPU_GyroscopeData(I2C_HandleTypeDef *i2c, sensor_t *veriler){
	int16_t gyro_raw[3];
	MPU_GetGyroscopeRawData(i2c, gyro_raw);
	//   2^(16)/2/250=131.072 LSB/(deg/s)  (+- 250 deg/s okudugumuz icin)
	//   1/131.072 LSB/(deg/s) = 0.0076293945 (deg/s)/LSB
	// 	 0.0076293945 (deg/s)/LSB / 180 deg* 3.14159 rad  = 0.0001332 (rad/s)/LSB
	veriler->gyro[0]=(float)(gyro_raw[0]- 665)*0.0001332;//665
	veriler->gyro[1]=(float)(gyro_raw[1]-  79)*0.0001332;//79
	veriler->gyro[2]=(float)(gyro_raw[2]- 256)*0.0001332;//256
	//radyan/s olarak cikti verir
}

//******************QCM FUNC*****************

HAL_StatusTypeDef QCM_init_config(I2C_HandleTypeDef *i2c ){
    uint8_t set_rst  = 0x01;   // Register 0x0B, önerilen değer
    uint8_t ctrl1    = 0xD1;   // OSR=512, RNG=±8G, ODR=200Hz, Mode=Continuous

	HAL_Delay(180);//normally we should wait 50 ms for sensor to power up
	HAL_StatusTypeDef ret         = HAL_I2C_IsDeviceReady(i2c,QCM5883L_ADDRESS,3,100);
	HAL_StatusTypeDef ret_set_rst = HAL_I2C_Mem_Write(i2c,QCM5883L_ADDRESS,QCM5883L_SET_RESET_REG,1,&set_rst,1,10);
	HAL_StatusTypeDef ret_ctrl_1  = HAL_I2C_Mem_Write(i2c,QCM5883L_ADDRESS,QCM5883L_CTRL_REG_1   ,1,&ctrl1 ,1,10);
	if ( ret==HAL_OK && ret_ctrl_1==HAL_OK && ret_set_rst==HAL_OK  ){//0 olması islemin gerceklestirildigi anlamina gelir
	    return HAL_OK; // device is ready
			}
	else {
		return HAL_ERROR;

		}
}


void QCM_data (I2C_HandleTypeDef *i2c ,  sensor_t *veriler){
	uint8_t status_reg;
	uint8_t readings[6];
	int16_t compass[3];
	HAL_StatusTypeDef ret_i2c =HAL_I2C_Mem_Read(i2c, 0x1A  ,   0x06, 1, &status_reg , 1, 50);
	//if(ret_i2c == HAL_OK){return 13;}//this works
	if((status_reg & 0x01)==1){
		HAL_I2C_Mem_Read(i2c, 0x1A,  0x00, 1,  &readings, 6, 50);
		compass[0]= (int16_t)(readings[1]<<8) | readings[0];
		compass[1]= (int16_t)(readings[3]<<8) | readings[2];
		//compass[2]= (int16_t)(readings[5]<<8) | readings[4];
		veriler->psi_magneto= atan2f(compass[1],compass[0]);//in radian
		if (veriler->psi_magneto>0){
			veriler->psi_magneto=veriler->psi_magneto;
					}
		else{
			veriler->psi_magneto=6.283185 + veriler->psi_magneto;
					}
	}
	else {
		HAL_ERROR;
	}

}

//******************MATH FUNC*******************

int invertMatrix(float input[SIZE][SIZE], float output[SIZE][SIZE]) {
	float augmented[SIZE][2 * SIZE]; // Augmented matrix
    int i, j, k;

    // Initialize the augmented matrix (input matrix | identity matrix)
    for (i = 0; i < SIZE; i++) {
        for (j = 0; j < SIZE; j++) {
            augmented[i][j] = input[i][j];
            augmented[i][j + SIZE] = (i == j) ? 1.0 : 0.0; // Identity matrix
        }
    }

    // Perform Gauss-Jordan elimination
    for (i = 0; i < SIZE; i++) {
        // Check for zero pivot element
        if (fabs(augmented[i][i]) < 1e-9) {
            //fprintf(stderr, "Matrix is singular and cannot be inverted.\n");
            return -45;
        }

        // Normalize the pivot row
        float pivot = augmented[i][i];
        for (j = 0; j < 2 * SIZE; j++) {
            augmented[i][j] /= pivot;
        }

        // Eliminate other rows
        for (k = 0; k < SIZE; k++) {
            if (k != i) {
                float factor = augmented[k][i];
                for (j = 0; j < 2 * SIZE; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // Extract the inverse matrix from the augmented matrix
    for (i = 0; i < SIZE; i++) {
        for (j = 0; j < SIZE; j++) {
            output[i][j] = augmented[i][j + SIZE];
        }
    }

    return 20; // Success
}
//*********************************************************************************
//*************************************SF FUNC*************************************
//*********************************************************************************


void Sensor_Fusion(I2C_HandleTypeDef *i2c,  SF_KF_t *K_filter,  sensor_t *veriler ){
	MPU_GyroscopeData(i2c,     veriler);
	MPU_AccelerationData(i2c,  veriler);

	//indicators
	volatile float buffer_theta_accel;
	volatile float buffer_phi_accel;
	volatile float norm=0.0;
	volatile float norm2=0.0;
	volatile float norm3=0.0;
	volatile float result[4][4];
	volatile float result2[4][4];
	volatile float temp_vector[4];
	volatile float temp_vector2[4];
	volatile float DT=0.01;
	int8_t returned=0;
	int8_t error= 0;
    volatile float total_a=sqrt(pow(veriler->accel[0],2)+pow(veriler->accel[1],2)+pow(veriler->accel[2],2));
    volatile float sqrta=sqrt(pow(veriler->accel[0],2)+pow(veriler->accel[2],2));

	veriler->theta_accel = asin(veriler->accel[0]/total_a);
    veriler->phi_accel   = asin(-veriler->accel[1]/cos(veriler->theta_accel)/total_a);
	//veriler->theta_accel = atan2(-veriler->accel[0],   veriler->accel[2]);
    //veriler->phi_accel   = atan(veriler->accel[1]/sqrta);


    if (isnan(veriler->theta_accel)){veriler->theta_accel= buffer_theta_accel;}
    else 					  	    {buffer_theta_accel  = veriler->theta_accel;}
    if (isnan(veriler->phi_accel))  {veriler->phi_accel  = buffer_phi_accel;}
    else 					  		{buffer_phi_accel    = veriler->phi_accel;}

    K_filter->q_accel[0]=  cos(veriler->phi_accel/2)*cos(veriler->theta_accel/2)*cos(veriler->psi_magneto) + sin(veriler->phi_accel/2)*sin(veriler->theta_accel/2)*sin(veriler->psi_magneto);
    K_filter->q_accel[1]=  sin(veriler->phi_accel/2)*cos(veriler->theta_accel/2)*cos(veriler->psi_magneto) - cos(veriler->phi_accel/2)*sin(veriler->theta_accel/2)*sin(veriler->psi_magneto);
    K_filter->q_accel[2]=  cos(veriler->phi_accel/2)*sin(veriler->theta_accel/2)*cos(veriler->psi_magneto) + sin(veriler->phi_accel/2)*cos(veriler->theta_accel/2)*sin(veriler->psi_magneto);
    K_filter->q_accel[3]=  cos(veriler->phi_accel/2)*cos(veriler->theta_accel/2)*sin(veriler->psi_magneto) - sin(veriler->phi_accel/2)*sin(veriler->theta_accel/2)*cos(veriler->psi_magneto);

  	norm2= sqrt(pow(K_filter->q_accel[0],2)+pow(K_filter->q_accel[1],2)+pow(K_filter->q_accel[2],2)+pow(K_filter->q_accel[3],2));
  	K_filter->q_accel[0]=K_filter->q_accel[0]/norm2;
  	K_filter->q_accel[1]=K_filter->q_accel[1]/norm2;
  	K_filter->q_accel[2]=K_filter->q_accel[2]/norm2;
  	K_filter->q_accel[3]=K_filter->q_accel[3]/norm2;

    K_filter->q_pred[0]=(volatile float)(	   	 	             K_filter->q_est[0]  -0.5*DT*veriler->gyro[0]*K_filter->q_est[1] -0.5*DT*veriler->gyro[1]*K_filter->q_est[2]  -0.5*DT*veriler->gyro[2]*K_filter->q_est[3]);
    K_filter->q_pred[1]=(volatile float)(0.5*DT*veriler->gyro[0]*K_filter->q_est[0]  +			              K_filter->q_est[1] +0.5*DT*veriler->gyro[2]*K_filter->q_est[2]  -0.5*DT*veriler->gyro[1]*K_filter->q_est[3]);
    K_filter->q_pred[2]=(volatile float)(0.5*DT*veriler->gyro[1]*K_filter->q_est[0]  -0.5*DT*veriler->gyro[2]*K_filter->q_est[1] +	                      K_filter->q_est[2]  +0.5*DT*veriler->gyro[0]*K_filter->q_est[3]);
    K_filter->q_pred[3]=(volatile float)(0.5*DT*veriler->gyro[2]*K_filter->q_est[0]  +0.5*DT*veriler->gyro[1]*K_filter->q_est[1] -0.5*DT*veriler->gyro[0]*K_filter->q_est[2]  +	  	 	    		   K_filter->q_est[3]);

  	norm3= sqrt(pow(K_filter->q_pred[0],2)+pow(K_filter->q_pred[1],2)+pow(K_filter->q_pred[2],2)+pow(K_filter->q_pred[3],2));
  	K_filter->q_pred[0]=K_filter->q_pred[0]/norm3;
  	K_filter->q_pred[1]=K_filter->q_pred[1]/norm3;
  	K_filter->q_pred[2]=K_filter->q_pred[2]/norm3;
  	K_filter->q_pred[3]=K_filter->q_pred[3]/norm3;
    //result'u bosalt
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
        	result[i][j] = 0.0;}}
    volatile float A_sys_Model[4][4] = {
    	  	            {		       		      1,   -veriler->gyro[0] * DT / 2,  -veriler->gyro[1] * DT / 2, 	 -veriler->gyro[2] * DT / 2},
    	  	            {veriler->gyro[0] * DT / 2 ,	     					1,   veriler->gyro[2] * DT / 2, 	 -veriler->gyro[1] * DT / 2},
    	  	            {veriler->gyro[1] * DT / 2 ,   -veriler->gyro[2] * DT / 2,     	     	   			 1, 	  veriler->gyro[0] * DT / 2},
    	  	            {veriler->gyro[2] * DT / 2 ,    veriler->gyro[1] * DT / 2,  -veriler->gyro[0] * DT / 2,	   							  1}
    	  	        };
    volatile float A_sys_Model_Trnps[4][4] = {
    	  	            {		       		      1,     veriler->gyro[0] * DT / 2,    veriler->gyro[1] * DT / 2,   veriler->gyro[2] * DT / 2},
    	  	            {-veriler->gyro[0] * DT / 2, 				   		     1,   -veriler->gyro[2] * DT / 2,   veriler->gyro[1] * DT / 2},
    	  	            {-veriler->gyro[1] * DT / 2,     veriler->gyro[2] * DT / 2,                            1,  -veriler->gyro[0] * DT / 2},
    	  	            {-veriler->gyro[2] * DT / 2,    -veriler->gyro[1] * DT / 2,    veriler->gyro[0] * DT / 2,	     					1}
    	  	        };
    //A*P_k_upminus:  sistem modelini onceki P degeriyle carp ve result'a depola
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {for (int8_t k = 0; k < 4; k++) {
    	    result[i][j] =(volatile float) A_sys_Model[i][k] * K_filter->P[k][j];    }}}
    //(A*P)*A' transpoz ile carpma: result'ta depolanan matrisi sist modelinin transpozuyla carp
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {for (int8_t k = 0; k < 4; k++) {
    	    result[i][j] =(volatile float) result[i][k] * A_sys_Model_Trnps[k][j];   }}}
    //Q ekleme
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	result[i][j] += K_filter->Q[i][j];  }}
    //P_k_upminus'u hesapla
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	K_filter->P_k_upminus[i][j] = result[i][j];}}
    //K_gain
    //halen result P_k_upminus'u depoluyor islem yapabiliriz
    //P_k_minus+R
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	result[i][j] += K_filter->R[i][j];  }}


    //result2'yi bosalt
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
       	result2[i][j] = 0.0;}}
    returned =invertMatrix(result, result2);//(P_k_minus+R)^(-1) degeri result2'ye depolaniyor
    if (returned==20) {for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {for (int8_t k = 0; k < 4; k++) {
	                K_filter->K_gain[i][j] =(volatile float) K_filter->P_k_upminus[i][k] * result2[k][j];}}} //K_gain= P_kupminus(P_k_upminus+R)
	    error=1;}
    else  {for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	K_filter->K_gain[i][j] = 0.0;}}
    	error=-45;}


    //estimation
    //temp_vectoru bosalt
    for (int8_t i = 0; i < 4; i++) {
    	temp_vector[i] = 0.0;}
    //temp_vector2'yi bosalt
    for (int8_t i = 0; i < 4; i++) {
    	temp_vector2[i] = 0.0;}
    //z-x //vector-vector
    for (int8_t i = 0; i < 4; i++) {
    	temp_vector[i] = K_filter->q_accel[i] - K_filter->q_pred[i];}
    //matrix*vector
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	temp_vector2[i] += K_filter->K_gain[i][j] * temp_vector[j];}}
    //temp_vectoru bosalt ve q_pred'i depola
    for (int8_t i = 0; i < 4; i++) {
    	temp_vector[i] =0.0;
    	temp_vector[i] =K_filter->q_pred[i];   }
    //temp_vectoru bosalt ve q_pred'i depola
    for (int8_t i = 0; i < 4; i++) {
    	temp_vector[i] =0.0;
    	temp_vector[i] =K_filter->q_pred[i];}
    //estimation quaternion
    for (int8_t i = 0; i < 4; i++) {//vector-vector
    	//q_est[i] =0.0;
    	K_filter->q_est[i] =temp_vector[i]+ temp_vector2[i];}


    //result'u ve result2'yi bosalt ve P_k_upminus'i result'a depola
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
        result[i][j] = 0.0;
        result2[i][j] = 0.0;
        result[i][j] =K_filter->P_k_upminus[i][j] ;}}
    //K_gain*P_k_upminus
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {for (int8_t k = 0; k < 4; k++) {
        result2[i][j] = K_filter->K_gain[i][k] * result[k][j];}}}
    for (int8_t i = 0; i < 4; i++) {for (int8_t j = 0; j < 4; j++) {
    	K_filter->P[i][j] =K_filter->P_k_upminus[i][j]-result2[i][j] ;}}
  	norm= sqrt(pow(K_filter->q_est[0],2)+pow(K_filter->q_est[1],2)+pow(K_filter->q_est[2],2)+pow(K_filter->q_est[3],2));
  	K_filter->q_est[0]=K_filter->q_est[0]/norm;
  	K_filter->q_est[1]=K_filter->q_est[1]/norm;
  	K_filter->q_est[2]=K_filter->q_est[2]/norm;
  	K_filter->q_est[3]=K_filter->q_est[3]/norm;

  	veriler->THETA = -atan2( (   K_filter->q_est[2]*K_filter->q_est[3] + K_filter->q_est[0]*K_filter->q_est[1]   )*2 , 1 - 2*( pow(K_filter->q_est[1],2) + pow(K_filter->q_est[2],2) )   )*57.2957795131;
  	veriler->PHI   =   asin(  (   K_filter->q_est[1]*K_filter->q_est[3] - K_filter->q_est[0]*K_filter->q_est[2]   )*2    )*57.2957795131 ;
  	//veriler->PSI   =  atan2( (   K_filter->q_est[1]*K_filter->q_est[2] + K_filter->q_est[0]*K_filter->q_est[3]   )*2 , 1 - 2*( pow(K_filter->q_est[2],2) + pow(K_filter->q_est[3],2) )   )*57.2957795131;
  	//norm2= atan2(2*( K_filter->q_est[0]*K_filter->q_est[1] + K_filter->q_est[2]*K_filter->q_est[3]), 1-2*(pow(K_filter->q_est[1],2) + pow(K_filter->q_est[2],2) ));
  	//norm3= -3.14159/4+ 2*atan2(sqrt(1+2*(K_filter->q_est[0]*K_filter->q_est[2]-K_filter->q_est[1]*K_filter->q_est[3]   )   ), sqrt(1-2*(K_filter->q_est[0]*K_filter->q_est[2]  -  K_filter->q_est[1]*K_filter->q_est[3]    ))   );
  	//veriler->THETA= norm3/3.14159*180;
  	//veriler->PHI= norm2*180/3.14159;
  /*	phi   =  atan2( 2*(x(3)*x(4) + x(1)*x(2)) , 1 - 2*(x(2)^2 + x(3)^2) );
  	theta = -asin(  2*(x(2)*x(4) - x(1)*x(3)) );
  	psi   =  atan2( 2*(x(2)*x(3) + x(1)*x(4)), 1 - 2*(x(3)^2 + x(4)^2) );*/

}
