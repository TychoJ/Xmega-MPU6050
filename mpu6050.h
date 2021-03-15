/*!
 *  \file    mpu6050.h
 *  \author  Tycho Jobsis
 *  \date    16-04-2018
 *  \version 0.1.0
 *
 *  \brief   MPU6050 library for the Xmega with the <a href="https://github.com/TychoJ/Xmega-TWI">Xmega-TWI library</a> 
 *
 *  \details This library is used to communicate with an MPU6050 sensor over I2C.
 *
 *	\note For the HvA-Xmegaboard you need to use TWIE!
 *
 *	\note This library <b>does not</b> support slave devices for the mpu6050!!! 
 */

/*!	\copyright
 *
 *	MIT License
 *
 *	Copyright (c) 2021 TychoJ
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 */


#include <avr/io.h>
#include <float.h>


#include "../TWI/TWI.h"

#ifndef MPU6050_H_
#define MPU6050_H_

/*!
*	\warning Do not change unless you do not use the HvA-Xmegaboard!!
*/
#define TWI_MODULE &TWIE

//0x68 (GND on AD0)
//0x69 (VCC on AD0)
#define MPU6050_ADDRESS 0x68

/*!
*	\brief Register definitions
*/

/*!
 *	\defgroup Self test registers
 */
#define MPU_6050_SELF_TEST_X	0x0D
#define MPU_6050_SELF_TEST_Y	0x0E
#define MPU_6050_SELF_TEST_Z	0x0F
#define MPU_6050_SELF_TEST_A	0x10


#define MPU_6050_SMPLRT_DIV		0x19

/*!
 *	\defgroup Sensor configuration registers of the MPU6050
 */
#define MPU_6050_CONFIG			0x1A
#define MPU_6050_GYRO_CONFIG	0x1B
#define MPU_6050_ACCEL_CONFIG	0x1C

/*!
 *	\defgroup slave settings for slave devices for the MPU6050
 */
#define MPU_6050_I2C_SLV0_ADDR	0x25
#define MPU_6050_I2C_SLV0_REG	0x26
#define MPU_6050_I2C_SLV0_DO	0x63
#define MPU_6050_I2C_SLV0_CTRL	0x27

#define MPU_6050_I2C_SLV1_ADDR	0x28
#define MPU_6050_I2C_SLV1_REG	0x29
#define MPU_6050_I2C_SLV1_DO	0x64
#define MPU_6050_I2C_SLV1_CTRL	0x2A

#define MPU_6050_I2C_SLV2_ADDR	0x2B
#define MPU_6050_I2C_SLV2_REG	0x2C
#define MPU_6050_I2C_SLV2_DO	0x65
#define MPU_6050_I2C_SLV2_CTRL	0x2D

#define MPU_6050_I2C_SLV3_ADDR	0x2E
#define MPU_6050_I2C_SLV3_REG	0x2F
#define MPU_6050_I2C_SLV3_DO	0x66
#define MPU_6050_I2C_SLV3_CTRL	0x302

#define MPU_6050_I2C_SLV4_ADDR	0x31
#define MPU_6050_I2C_SLV4_REG	0x32
#define MPU_6050_I2C_SLV4_DO	0x33
#define MPU_6050_I2C_SLV4_CTRL	0x34
#define MPU_6050_I2C_SLV4_DI	0x53


/*!
 *	\defgroup Master I2C registers for the MPU6050
 */
#define MPU_6050_I2C_MST_CTRL	0x24
#define MPU_6050_I2C_MST_STATUS	0x36
#define MPU_6050_I2C_MST_DELAY_CTRL	0x67

/*!
 *	\defgroup Interrupt registers
 */
#define MPU_6050_INT_PIN_CFG	0x37
#define MPU_6050_INT_ENABLE		0x38
#define MPU_6050_INT_STATUS		0x3A

/*!
 *	\defgroup Accelerometer output registers
 */
#define MPU_6050_ACCEL_XOUT_H	0x3B
#define MPU_6050_ACCEL_XOUT_L	0x3C
#define MPU_6050_ACCEL_YOUT_H	0x3D
#define MPU_6050_ACCEL_YOUT_L	0x3E
#define MPU_6050_ACCEL_ZOUT_H	0x3F
#define MPU_6050_ACCEL_ZOUT_L	0x40

/*!
 *	\defgroup Temperatures output registers
 */
#define MPU_6050_TEMP_OUT_H		0x41
#define MPU_6050_TEMP_OUT_L		0x42

/*!
 *	\defgroup Gyroscope output registers
 */
#define MPU_6050_GYRO_XOUT_H	0x43
#define MPU_6050_GYRO_XOUT_L	0x44
#define MPU_6050_GYRO_YOUT_H	0x45
#define MPU_6050_GYRO_YOUT_L	0x46
#define MPU_6050_GYRO_ZOUT_H	0x47
#define MPU_6050_GYRO_ZOUT_L	0x48

/*!
 *	\defgroup external sensor data registers
 */
#define MPU_6050_EXT_SENS_DATA_00	0x49
#define MPU_6050_EXT_SENS_DATA_01	0x4A
#define MPU_6050_EXT_SENS_DATA_02	0x4B
#define MPU_6050_EXT_SENS_DATA_03	0x4C
#define MPU_6050_EXT_SENS_DATA_04	0x4D
#define MPU_6050_EXT_SENS_DATA_05	0x4E
#define MPU_6050_EXT_SENS_DATA_06	0x4F
#define MPU_6050_EXT_SENS_DATA_07	0x50
#define MPU_6050_EXT_SENS_DATA_08	0x51
#define MPU_6050_EXT_SENS_DATA_09	0x52
#define MPU_6050_EXT_SENS_DATA_10	0x53
#define MPU_6050_EXT_SENS_DATA_11	0x54
#define MPU_6050_EXT_SENS_DATA_12	0x55
#define MPU_6050_EXT_SENS_DATA_13	0x56
#define MPU_6050_EXT_SENS_DATA_14	0x57
#define MPU_6050_EXT_SENS_DATA_15	0x58
#define MPU_6050_EXT_SENS_DATA_16	0x59
#define MPU_6050_EXT_SENS_DATA_17	0x5A
#define MPU_6050_EXT_SENS_DATA_18	0x5B
#define MPU_6050_EXT_SENS_DATA_19	0x5C
#define MPU_6050_EXT_SENS_DATA_20	0x5D
#define MPU_6050_EXT_SENS_DATA_21	0x5E
#define MPU_6050_EXT_SENS_DATA_22	0x5F
#define MPU_6050_EXT_SENS_DATA_23	0x60


#define MPU_6050_SIGNAL_PATH_RESET	0x68


#define MPU_6050_USER_CTRL	0x6A

/*!
 *	\defgroup Power management registers
 */
#define MPU_6050_PWR_MGMT_1	0x6B
#define MPU_6050_PWR_MGMT_2	0x6C

/*!
 *	\defgroup FIFO registers of the MPU6050
 */
#define MPU_6050_FIFO_EN		0x23
#define MPU_6050_FIFO_COUNTH	0x72
#define MPU_6050_FIFO_COUNTL	0x73
#define MPU_6050_FIFO_R_W		0x74


#define MPU_6050_WHO_AM_I	0x752

/*!
 *	\defgroup Bit locations in register
 */
#define DATA_RDY_INT_EN			0
#define MPU_6050_I2C_MST_INT_EN	3
#define MPU_6050_FIFO_INT_EN	4

/*!
 *	\defgroup return values for what_happened_MPU6050
 */
#define DATA_RDY_INT			3
#define MPU_6050_I2C_MST_INT	4
#define MPU_6050_FIFO_INT		5

/*!
 *	\def MPU6050_CLK_8MHZ
 *	CLK selection value
 */
#define MPU6050_CLK_8MHZ 0

/*!
 *	\def MPU6050_TWI_ERROR
 *	Status code for TWI
 */
/*!
 *	\def MPU6050_TWI_OK
 *	Status code for TWI
 */
#define MPU6050_TWI_ERROR	1
#define MPU6050_TWI_OK		0

#define MPU6050_ACCEL_SCL_2G	0	//!< +-2G Max measurement
#define MPU6050_ACCEL_SCL_4G	1	//!< +-4G Max measurement
#define MPU6050_ACCEL_SCL_8G	2	//!< +-8G Max measurement
#define MPU6050_ACCEL_SCL_16G	3	//!< +-16G Max measurement

#define MPU6050_GYRO_SCL_250	0	//!< +-250 degrees per second Max measurement 
#define MPU6050_GYRO_SCL_500	1	//!< +-500 degrees per second Max measurement 
#define MPU6050_GYRO_SCL_1000	2	//!< +-1000 degrees per second Max measurement 
#define MPU6050_GYRO_SCL_2000	3	//!< +-2000 degrees per second Max measurement 


/*! \brief  Union to store 16bits sensor values 
 *
 *	This union is used to store a 16 bit sensor value in two 8bit unsigned integers.
 *	You can use DATA to access the whole 16 bit value. You can also use DATAL and DATAH to either store the lower or the higher eight bits of the sixteen bit DATA value.
 *  
 *
 */
typedef union {
	struct {
		uint8_t DATAL;
		uint8_t DATAH;
	};
	int16_t DATA;
} data16_t;

typedef union {
	struct {
		uint8_t DATAL;
		uint8_t DATAH;
	};
	int16_t TEMP;
} TEMP16_t;


uint8_t enable_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t disable_mpu6050(TWI_t *twi, uint8_t addr);

uint8_t wake_up_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t sleep_mpu6050(TWI_t *twi, uint8_t addr);

uint8_t get_accel_x_mpu6050(TWI_t *twi, uint8_t addr, float *data);
uint8_t get_accel_y_mpu6050(TWI_t *twi, uint8_t addr, float *data);
uint8_t get_accel_z_mpu6050(TWI_t *twi, uint8_t addr, float *data);

uint8_t get_gyro_x_mpu6050(TWI_t *twi, uint8_t addr, float *data);
uint8_t get_gyro_y_mpu6050(TWI_t *twi, uint8_t addr, float *data);
uint8_t get_gyro_z_mpu6050(TWI_t *twi, uint8_t addr, float *data);

uint8_t get_temp_mpu6050(TWI_t *twi, uint8_t addr, float *data);

uint8_t int_enable_mpu6050(TWI_t *twi, uint8_t addr, uint8_t interupt);
uint8_t int_disable_mpu6050(TWI_t *twi, uint8_t addr, uint8_t interupt);
uint8_t what_happend_mpu6050(TWI_t *twi, uint8_t addr);

uint8_t ext_sens_value_mpu6050(TWI_t *twi, uint8_t addr, uint8_t reg, uint8_t *data);

uint8_t disable_temp_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t enable_temp_mpu6050(TWI_t *twi, uint8_t addr);

uint8_t reset_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t reset_accel_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t reset_gyro_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t reset_temp_mpu6050(TWI_t *twi, uint8_t addr);

uint8_t clk_sel_mpu6050(TWI_t *twi, uint8_t addr, uint8_t clk_sel);

uint8_t self_test_x_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t self_test_y_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t self_test_z_mpu6050(TWI_t *twi, uint8_t addr);
uint8_t self_test_a_mpu6050(TWI_t *twi, uint8_t addr, uint8_t xyz);

uint8_t check_err_mpu6050(uint8_t err);

uint8_t accel_set_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t scale);
uint8_t accel_get_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t *scale);
uint8_t gyro_set_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t scale);
uint8_t gyro_get_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t *scale);
uint8_t temp_set_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t scale);
uint8_t temp_get_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t *scale);

#endif /* MPU6050_H_ */