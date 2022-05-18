/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: mpu.cpp
 *
 * Description: This code is for operating MPU6050 in dmp mode
 * This is main function in application layer.
 *
 * Author: HuyNS
 *
 * Last Changed By:  $Author: huyns $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Mar 24, 2022
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MPU6050/helper_3dmath.h"
#include "MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "MPU6050-App.h"



/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

#ifdef _GNUC_

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2 ,(uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
bool_t g_boDmpReady = false;  // set true if DMP init was successful
u8_t g_byDevStatus;
//u8_t g_g_byMpuIntStatus;   // holds actual interrupt status byte from MPU
u16_t g_wPacketSize;    // expected DMP packet size (default is 42 bytes)
u16_t g_wFifoCount;     // count of all bytes currently in FIFO
uint8_t g_byFifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion g_q;           // [w, x, y, z]         quaternion container
VectorInt16 g_Aa;         // [x, y, z]            accel sensor measurements
VectorInt16 g_AaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 g_aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat g_gravity;    // [x, y, z]            gravity vector
//float_t g_euler[3];         // [psi, theta, phi]    Euler angle container
float_t g_ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//VectorInt16 g_vInt16Gyro;
//VectorInt16 g_vInt16Accel;

MPU6050 g_mpu;
//interrupt service routine
volatile bool_t g_boMpuInterrupt = false;
u8_t g_byMpuIntStatus;


//Calib data
i32_t g_iwBuffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
i32_t g_iwAcel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
i32_t g_iwGiro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

i16_t g_iwAx, g_iwAy, g_iwAz, g_iwGx, g_iwGy, g_iwGz;
i32_t g_iwMean_ax, g_iwMean_ay, g_iwMean_az, g_iwMean_gx, g_iwMean_gy, g_iwMean_gz, g_iwState = 0;
i32_t g_iwAx_offset, g_iwAy_offset, g_iwAz_offset, g_iwGx_offset, g_iwGy_offset, g_iwGz_offset;



/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

static u8_t readDMP(void_t);
static void_t calibSetup(void_t);
static void_t meansensors(void_t);
static void_t calibration(void_t);
static void_t calibration_caloffset(void_t);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

/**
* @brief 	Check dmp status
*
* @param 		-		None
*
* @return		-	 	None
*/
void_t MPU_dmpDataReady(void_t) {
	g_boMpuInterrupt = TRUE;
}

 /**
  * @brief 	Initialize MPU sensor.
  *
  * @param 	[iwGyroXOffset]		:		gyroX offset
  * @param 	[iwGyroYOffset]		:		gyroY offset
  * @param 	[iwGyroZOffset]		:		gyroZ offset
  * @param 	[iwAccelZOffset]	:		Accel Z offset
  * @return		:	 	1 if success and 0 if fail
  */
u8_t MPU_Init(I2C_HandleTypeDef* hi2c)
{
	I2Cdev_init(hi2c);
	g_mpu.initialize();

	// load and configure the DMP
	g_byDevStatus = g_mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (g_byDevStatus == 0) {

		calibSetup();

		calibration_caloffset();
//		//supply your own gyro offsets here, scaled for min sensitivity
//		g_mpu.setXGyroOffset(g_iwGx_offset);
//		g_mpu.setYGyroOffset(g_iwGy_offset);
//		g_mpu.setZGyroOffset(g_iwGz_offset);
//		g_mpu.setZAccelOffset(g_iwAz_offset); // 1688 factory default for my test chip

//		g_mpu.setXGyroOffset(110);
//		g_mpu.setYGyroOffset(98);
//		g_mpu.setZGyroOffset(47);
//		g_mpu.setZAccelOffset(1236); // 1688 factory default for my test chip

		// turn on the DMP, now that it's ready
		g_mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection

		//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		g_byMpuIntStatus = g_mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		g_boDmpReady = true;

		// get expected DMP packet size for later comparison
		g_wPacketSize = g_mpu.dmpGetFIFOPacketSize();
		return MPU_SUCCESS;
	}
	else
	{
	  return MPU_FAIL;
	}
}

/**
* @brief 	Calculate DMP data
*
* @param 		-		None
*
* @return		-	 	None
*/
static u8_t readDMP(void_t)
{
	// if programming failed, don't try to do anything
	  if (!g_boDmpReady) return READ_MPU_DMP_FAIL;

	  // wait for MPU interrupt or extra packet(s) available
	  while (!g_boMpuInterrupt && g_wFifoCount < g_wPacketSize) {
		  // other program behavior stuff here
		  // .
		  // .
		  // .
		  // if you are really paranoid you can frequently test in between other
		  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
		  // while() loop to immediately process the MPU data
		  // .
		  // .
		  // .
	  }

	  // reset interrupt flag and get INT_STATUS byte
	  g_boMpuInterrupt = false;
	  g_byMpuIntStatus = g_mpu.getIntStatus();

	  // get current FIFO count
	  g_wFifoCount = g_mpu.getFIFOCount();

	  // check for overflow (this should never happen unless our code is too inefficient)
	  if ((g_byMpuIntStatus & 0x10) || g_wFifoCount == 1024) {
		  // reset so we can continue cleanly
		  g_mpu.resetFIFO();
	  // otherwise, check for DMP data ready interrupt (this should happen frequently)
	  } else if (g_byMpuIntStatus & 0x02) {
		  // wait for correct available data length, should be a VERY short wait
		  while (g_wFifoCount < g_wPacketSize) g_wFifoCount = g_mpu.getFIFOCount();

		  // read a packet from FIFO
		  g_mpu.getFIFOBytes(g_byFifoBuffer, g_wPacketSize);

		  // track FIFO count here in case there is > 1 packet available
		  // (this lets us immediately read more without waiting for an interrupt)
		  g_wFifoCount -= g_wPacketSize;

		  #ifdef OUTPUT_READABLE_QUATERNION
			  // display quaternion values in easy matrix form: w x y z
			  g_mpu.dmpGetQuaternion(&g_q, g_byFifoBuffer);
//			  g_mpu.dmpGetGyro(&g_vInt16Gyro, g_byFifoBuffer);
//
//			  g_mpu.dmpGetAccel(&g_Aa, g_byFifoBuffer);
//			  g_mpu.dmpGetGravity(&g_gravity, &g_q);
//			  g_mpu.dmpGetLinearAccel(&g_AaReal, &g_Aa, &g_gravity);
//			  g_mpu.dmpGetLinearAccelInWorld(&g_vInt16Accel, &g_AaReal, &g_q);
		  #endif

		  #ifdef OUTPUT_READABLE_EULER
			  // display Euler angles in degrees
			  mpu.dmpGetQuaternion(&q, fifoBuffer);
			  mpu.dmpGetEuler(euler, &q);
			  Serial.print("euler\t");
			  Serial.print(euler[0] * 180/M_PI);
			  Serial.print("\t");
			  Serial.print(euler[1] * 180/M_PI);
			  Serial.print("\t");
			  Serial.println(euler[2] * 180/M_PI);
		  #endif

		  #ifdef OUTPUT_READABLE_YAWPITCHROLL
			  // display Euler angles in degrees
			  mpu.dmpGetQuaternion(&q, fifoBuffer);
			  mpu.dmpGetGravity(&gravity, &q);
			  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			  Serial.print("ypr\t");
			  Serial.print(ypr[0] * 180/M_PI);
			  Serial.print("\t");
			  Serial.print(ypr[1] * 180/M_PI);
			  Serial.print("\t");
			  Serial.println(ypr[2] * 180/M_PI);
		  #endif

		  #ifdef OUTPUT_READABLE_REALACCEL
			  // display real acceleration, adjusted to remove gravity
			  mpu.dmpGetQuaternion(&q, fifoBuffer);
			  mpu.dmpGetAccel(&aa, fifoBuffer);
			  mpu.dmpGetGravity(&gravity, &q);
			  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			  Serial.print("areal\t");
			  Serial.print(aaReal.x);
			  Serial.print("\t");
			  Serial.print(aaReal.y);
			  Serial.print("\t");
			  Serial.println(aaReal.z);
		  #endif

		  #ifdef OUTPUT_READABLE_WORLDACCEL
			  // display initial world-frame acceleration, adjusted to remove gravity
			  // and rotated based on known orientation from quaternion
			  mpu.dmpGetQuaternion(&q, fifoBuffer);
			  mpu.dmpGetAccel(&aa, fifoBuffer);
			  mpu.dmpGetGravity(&gravity, &q);
			  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			  Serial.print("aworld\t");
			  Serial.print(aaWorld.x);
			  Serial.print("\t");
			  Serial.print(aaWorld.y);
			  Serial.print("\t");
			  Serial.println(aaWorld.z);
		  #endif

		  #ifdef OUTPUT_TEAPOT
			  // display quaternion values in InvenSense Teapot demo format:
			  teapotPacket[2] = fifoBuffer[0];
			  teapotPacket[3] = fifoBuffer[1];
			  teapotPacket[4] = fifoBuffer[4];
			  teapotPacket[5] = fifoBuffer[5];
			  teapotPacket[6] = fifoBuffer[8];
			  teapotPacket[7] = fifoBuffer[9];
			  teapotPacket[8] = fifoBuffer[12];
			  teapotPacket[9] = fifoBuffer[13];
			  Serial.write(teapotPacket, 14);
			  teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		  #endif
		  return READ_MPU_SUCCESS;
	  }
	  else
	  {
		  return READ_MPU_I2C_LOST;
	  }
}

/**
 * @brief 	Get Quaternion
 *
 * @param 	[&q]		:		return result
 *
 * @return	[status]	:		status
 */
u8_t MPU_GetQuaternion(Quaternion_t &q){
	u8_t status = readDMP();
	q.w = g_q.w;
	q.x = g_q.x;
	q.y = g_q.y;
	q.z = g_q.z;
	return status;
}

/**
 * @brief 	Get Accel, gyro
 *
 * @param 	[&a]		:		return result
 *
 * @return	[status]	:		status
 */
u8_t MPU_GetAccel_Gyro(Accel_gyro_t &a)
{
	u8_t status = readDMP();
	a.accelX = g_mpu.getAccelerationX()/16384.0;
	a.accelY = g_mpu.getAccelerationY()/16384.0;
	a.accelZ = g_mpu.getAccelerationZ()/16384.0;

	a.gyroX = g_mpu.getRotationX()/16.4;
	a.gyroY = g_mpu.getRotationY()/16.4;
	a.gyroZ = g_mpu.getRotationZ()/16.4;

	return status;
}

/**
 * @brief 	Get All DMP value, include quaternion and Accel, gyro
 *
 * @param 	[&]		:		return result
 *
 * @return	[status]	:		status
 */
u8_t MPU_GetDMPValue(DMP_data_t &d)
{
	u8_t status = readDMP();

	d.qw = g_q.w;
	d.qx = g_q.x;
	d.qy = g_q.y;
	d.qz = g_q.z;

	d.accelX = g_mpu.getAccelerationX()/16384.0;
	d.accelY = g_mpu.getAccelerationY()/16384.0;
	d.accelZ = g_mpu.getAccelerationZ()/16384.0;

	d.gyroX = g_mpu.getRotationX()/16.4;
	d.gyroY = g_mpu.getRotationY()/16.4;
	d.gyroZ = g_mpu.getRotationZ()/16.4;

	return status;
}

/**
 * @brief 	Calculate data
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
//u8_t MPU_GetEuler(EulerAngle_t &e){
//	u8_t status = readDMP();
//	g_mpu.dmpGetEuler(e, &g_q);
//	return status;
//}

/**
 * @brief 	Calculate yaw, pitch, roll
 *
 * @param 	[&y]		:		return result
 *
 * @return	[status]	:		status
 */
u8_t MPU_GetYawPitchRoll(YPRAngle_t &y){
	u8_t status = readDMP();
	g_mpu.dmpGetGravity(&g_gravity, &g_q);
	g_mpu.dmpGetYawPitchRoll(g_ypr, &g_q, &g_gravity);

	y.yaw = g_ypr[0]*180/M_PI;
	y.pitch = g_ypr[1]*180/M_PI;
	y.roll = g_ypr[2]*180/M_PI;

	return status;
}



/**
 * @brief 	Calculate data
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
//u8_t MPU_GetLinearAccel(void_t);
/**
 * @brief 	Calculate data
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
//VectorInt16 MPU_GetLinearAccelInWorld(void_t);


/**
 * @brief 	setup default for calib
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
static void_t calibSetup(void_t)
{
	g_mpu.setXAccelOffset(0);
	g_mpu.setYAccelOffset(0);
	g_mpu.setZAccelOffset(0);
	g_mpu.setXGyroOffset(0);
	g_mpu.setYGyroOffset(0);
	g_mpu.setZGyroOffset(0);
}

/**
 * @brief 	read mean value
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
static void_t meansensors(void_t){
  i64_t idw_i = 0, idwBuff_ax=0, idwBuff_ay=0, idwBuff_az=0, idwBuff_gx=0, idwBuff_gy=0, idwBuff_gz=0;

  while (idw_i < (g_iwBuffersize+101)){
    // read raw accel/gyro measurements from device
    g_mpu.getMotion6(&g_iwAx, &g_iwAy, &g_iwAz, &g_iwGx, &g_iwGy, &g_iwGz);

    if (idw_i>100 && idw_i<=(g_iwBuffersize+100)){ //First 100 measures are discarded
      idwBuff_ax = idwBuff_ax + g_iwAx;
      idwBuff_ay = idwBuff_ay + g_iwAy;
      idwBuff_az = idwBuff_az + g_iwAz;
      idwBuff_gx = idwBuff_gx + g_iwGx;
      idwBuff_gy = idwBuff_gy + g_iwGy;
      idwBuff_gz = idwBuff_gz + g_iwGz;
    }
    if (idw_i == (g_iwBuffersize + 100)){
    	g_iwMean_ax = idwBuff_ax / g_iwBuffersize;
    	g_iwMean_ay = idwBuff_ay / g_iwBuffersize;
    	g_iwMean_az = idwBuff_az / g_iwBuffersize;
    	g_iwMean_gx = idwBuff_gx / g_iwBuffersize;
    	g_iwMean_gy = idwBuff_gy / g_iwBuffersize;
    	g_iwMean_gz = idwBuff_gz / g_iwBuffersize;
    }
    idw_i++;
    HAL_Delay(2); //Needed so we don't get repeated measures
  }
}

/**
 * @brief 	Calibarate
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
static void_t calibration(void_t){
	g_iwAx_offset =- g_iwMean_ax / 8;
	g_iwAy_offset =- g_iwMean_ay / 8;
	g_iwAz_offset = (16384 - g_iwMean_az) / 8;

	g_iwGx_offset =- g_iwMean_gx / 4;
	g_iwGy_offset =- g_iwMean_gy / 4;
	g_iwGz_offset =- g_iwMean_gz / 4;
  while (1){
    i32_t iwReady = 0;
    g_mpu.setXAccelOffset(g_iwAx_offset);
    g_mpu.setYAccelOffset(g_iwAy_offset);
    g_mpu.setZAccelOffset(g_iwAz_offset);

    g_mpu.setXGyroOffset(g_iwGx_offset);
    g_mpu.setYGyroOffset(g_iwGy_offset);
    g_mpu.setZGyroOffset(g_iwGz_offset);

    meansensors();
    if (abs(g_iwMean_ax) <= g_iwAcel_deadzone) iwReady++;
    else g_iwAx_offset = g_iwAx_offset - g_iwMean_ax / g_iwAcel_deadzone;

    if (abs(g_iwMean_ay) <= g_iwAcel_deadzone) iwReady++;
    else g_iwAy_offset = g_iwAy_offset-g_iwMean_ay / g_iwAcel_deadzone;

    if (abs(16384-g_iwMean_az) <= g_iwAcel_deadzone) iwReady++;
    else g_iwAz_offset = g_iwAz_offset + (16384 - g_iwMean_az)/g_iwAcel_deadzone;

    if (abs(g_iwMean_gx)<=g_iwGiro_deadzone) iwReady++;
    else g_iwGx_offset = g_iwGx_offset - g_iwMean_gx / (g_iwGiro_deadzone + 1);

    if (abs(g_iwMean_gy)<=g_iwGiro_deadzone) iwReady++;
    else g_iwGy_offset = g_iwGy_offset - g_iwMean_gy / (g_iwGiro_deadzone + 1);

    if (abs(g_iwMean_gz)<=g_iwGiro_deadzone) iwReady++;
    else g_iwGz_offset=g_iwGz_offset-g_iwMean_gz / (g_iwGiro_deadzone+1);

    if (iwReady==6) break;
  }
}

/**
 * @brief 	Calculate data
 *
 * @param 		-		None
 *
 * @return		-	 	None
 */
static void_t calibration_caloffset(void_t)
{
	if(g_iwState == 0)
	{
		meansensors();
		g_iwState++;
		HAL_Delay(1000);
	}
	if(g_iwState == 1)
	{
		calibration();
		g_iwState++;
		HAL_Delay(1000);
	}
	if(g_iwState == 2)
	{
		meansensors();
	}
}
