/*
 * MPU6050-App.h
 *
 *  Created on: Mar 24, 2022
 *      Author: hi
 */

#ifndef SOURCE_APP_MPU6050_APP_MPU6050_APP_H_
#define SOURCE_APP_MPU6050_APP_MPU6050_APP_H_



/* Last Changed By:  $Author: huyns $
* Revision:         $Revision: $
* Last Changed:     $Date: $Mar 24, 2022
*
* Code sample:
******************************************************************************/
// Enclosing macro to prevent multiple inclusion

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "MPU6050/MPU6050.h"
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
enum{
	MPU_SUCCESS		= 	0x00,
	MPU_FAIL		=	0x01
};

enum{
	READ_MPU_SUCCESS	=	0,
	READ_MPU_OVERFLOW	=	1,
	READ_MPU_I2C_LOST	=	2,
	READ_MPU_DMP_FAIL	=	3
};

typedef struct {
	float_t w;
	float_t x;
	float_t y;
	float_t z;
}Quaternion_t;

typedef struct{
	float_t psi;
	float_t theta;
	float_t phi;
}EulerAngle_t;

typedef struct{
	float_t yaw;
	float_t pitch;
	float_t roll;
}YPRAngle_t;

typedef struct
{
	float_t gyroX;
	float_t gyroY;
	float_t gyroZ;
	float_t accelX;
	float_t accelY;
	float_t accelZ;
}Accel_gyro_t;

typedef struct
{
	float_t qw;
	float_t qx;
	float_t qy;
	float_t qz;

	float_t gyroX;
	float_t gyroY;
	float_t gyroZ;
	float_t accelX;
	float_t accelY;
	float_t accelZ;
}DMP_data_t;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
//

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void_t MPU_dmpDataReady(void_t);
u8_t MPU_GetQuaternion(Quaternion_t &q);
u8_t MPU_GetAccel_Gyro(Accel_gyro_t &a);
u8_t MPU_GetDMPValue(DMP_data_t &d);
//u8_t MPU_GetEuler(EulerAngle_t &e);
u8_t MPU_GetYawPitchRoll(YPRAngle_t &y);
u8_t MPU_Init(I2C_HandleTypeDef* hi2c);
/******************************************************************************/


#endif /* SOURCE_APP_MPU6050_APP_MPU6050_APP_H_ */
