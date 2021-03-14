/*!
 *  \file    mpu6050.c
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

#include <float.h>
#include "mpu6050.h"
#include "../TWI/TWI.h"


typedef union {
	struct {
		uint8_t DATA_RDY : 1;
		uint8_t : 2;
		uint8_t I2C_MST : 1;
		uint8_t FIFO_OFLOW : 1;
		uint8_t : 3;
	};
	uint8_t int_reg;
} MPU6050_INT_STATUS_TYPE;

typedef union {
	struct {
		uint8_t CLKSEL : 3;
		uint8_t TEMP_DIS : 1;
		uint8_t : 1;
		uint8_t CYCLE : 1;
		uint8_t MPU_SLEEP : 1;
		uint8_t DEVICE_RESET : 1;
	};
	uint8_t PWR_MGMT_1;
} MPU6050_PWR_MGMT_1_TYPE;

typedef union {
	struct {
		uint8_t : 3;
		uint8_t FS_SEL : 2;
		uint8_t ZG_ST : 1;
		uint8_t YG_ST : 1;
		uint8_t XG_ST : 1;
	};
	uint8_t GYRO_CONFIG;
} MPU6050_GYRO_CONFIG_TYPE;

typedef union {
	struct {
		uint8_t : 3;
		uint8_t AFS_SEL : 2;
		uint8_t ZA_ST : 1;
		uint8_t YA_ST : 1;
		uint8_t XA_ST : 1;
	};
	uint8_t ACCEL_CONFIG;
} MPU6050_ACCEL_CONFIG_TYPE;



/*! \brief  Checks for errors 
 *
 *	\note	This function is for internal use
 *
 *  \param  err		byte that is checked for error status
 *
 *  \return	0 if no errors occurred if TWI/I2C bus is in use returns 1 otherwise returns 2	
 */
uint8_t check_err_mpu6050(uint8_t err){
	if(err == BUS_IN_USE) return 1;
	if(err == NACK) return 2;
	if(err == DATA_NOT_SEND) return 2;
	if(err == DATA_NOT_RECEIVED) return 2;
	return 0;
}

/*! \brief  Enables the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return	0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2	
 */
uint8_t enable_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t old_reg, err; 
		
	err = wake_up_mpu6050(twi, addr);
	if(err != 0) return err;
	
	clk_sel_mpu6050(twi, addr, MPU6050_CLK_8MHZ);
	if(err != 0) return err;
	
	err = enable_temp_mpu6050(twi, addr);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	old_reg = 0; //!< Makes the Gyroscope and the accelerometer active
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_2);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return MPU6050_TWI_OK;	
}

/*! \brief  Disables the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t disable_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t old_reg, err;
	
	err = disable_temp_mpu6050(twi, addr);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	old_reg = 1; //!< Makes the Gyroscope and the accelerometer inactive
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_2);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	err = sleep_mpu6050(twi, addr);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Wakes the MPU6050 from sleep
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t wake_up_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err, old_reg;
	
	err = read_8bit_register_TWI(twi, addr, &old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	old_reg = (0 << 6); //!< Disables sleep
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Lets the MPU6050 go to sleep
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t sleep_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err, old_reg;
	
	err = read_8bit_register_TWI(twi, addr, &old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	old_reg = (1 << 6); //!< Enables sleep
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Get x-axis accelerometer data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store accelerometer x direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_accel_x_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t accel_x;
	accel_x.DATA = 0;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_XOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_x.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_XOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_x.DATAH = read;
	
	(*data) = accel_x.DATA;
	
	return 0;
}

/*! \brief  Get y-axis accelerometer data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store accelerometer y direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_accel_y_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t accel_y;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_YOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_y.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_YOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_y.DATAH = read;
	
	(*data) = accel_y.DATA;
	
	return 0;	
}

/*! \brief  Get z-axis accelerometer data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store accelerometer z direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_accel_z_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t accel_z;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_ZOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_z.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_ACCEL_ZOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	accel_z.DATAH = read;
	
	(*data) = accel_z.DATA;
	
	return 0;	
}

/*! \brief  Get x-axis gyroscope data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store gyroscope x direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_gyro_x_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t gyro_x;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_XOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_x.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_XOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_x.DATAH = read;
	
	(*data) = gyro_x.DATA;
	
	return 0;	
}

/*! \brief  Get y-axis gyroscope data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store gyroscope y direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_gyro_y_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t gyro_y;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_YOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_y.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_YOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_y.DATAH = read;
	
	(*data) = gyro_y.DATA;
	
	return 0;	
}

/*! \brief  Get z-axis gyroscope data
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store gyroscope z direction data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_gyro_z_mpu6050(TWI_t *twi, uint8_t addr, int16_t *data){
	uint8_t err, read;
	data16_t gyro_z;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_ZOUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_z.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_GYRO_ZOUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	gyro_z.DATAH = read;
	
	(*data) = gyro_z.DATA;
	
	return 0;	
}

/*! \brief  Get temperature
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	data	pointer to store temperature data
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t get_temp_mpu6050(TWI_t *twi, uint8_t addr, float *data){
	uint8_t err, read;
	//int8_t readh;
	TEMP16_t temp;
	float ret;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_TEMP_OUT_L);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	temp.DATAL = read;
	
	err = read_8bit_register_TWI(twi, addr, &read, MPU_6050_TEMP_OUT_H);
	if (check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	temp.DATAH = (read << 0);
	
	//ret = ( (temp.TEMP / (float)340) + 36.53);
	ret = temp.TEMP;
	
	ret = (ret  / 340) + 36.53;
	
	(*data) = ret;
	
	return 0;	
}

/*! \brief  Enables interrupts from the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	interupt	byte to select what interrupt will be enabled
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t int_enable_mpu6050(TWI_t *twi, uint8_t addr, uint8_t interupt){
	uint8_t reg_old, err;
	
	err = read_8bit_register_TWI(twi, addr, &reg_old, MPU_6050_INT_ENABLE);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	reg_old = (1 << interupt);
	
	err = write_8bit_register_TWI(twi, addr, reg_old, MPU_6050_INT_ENABLE);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Disables interrupts from the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	interupt	byte to select what interrupt will be disabled
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t int_disable_mpu6050(TWI_t *twi, uint8_t addr, uint8_t interupt){
	uint8_t reg_old, err;
	
	err = read_8bit_register_TWI(twi, addr, &reg_old, MPU_6050_INT_ENABLE);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	reg_old = (0 << interupt);
	
	err = write_8bit_register_TWI(twi, addr, reg_old, MPU_6050_INT_ENABLE);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  checks what caused the interrupt from the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return returns 1 if TWI/I2C bus is in use returns 2 if something went wrong. 
 *			returns 3 if FIFO_OVF_INT is 1
 *			returns 4 if I2C_MST_INT is 1
 *			returns 5 if DATA_RDY_INT is 1
 *			returns 7 if FIFO_OVF_INT is 1 and I2C_MST_INT is 1
 *			returns 8 if FIFO_OVF_INT is 1 and DATA_RDY_INT is 1
 *			returns 9 if I2C_MST_INT is 1 and DATA_RDY_INT is 1
 *			returns 12 if FIFO_OVF_INT is 1 and I2C_MST_INT is 1 and DATA_RDY_INT is 1
 */
uint8_t what_happend_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err, ret = 0;
	MPU6050_INT_STATUS_TYPE int_status;
	
	err = read_8bit_register_TWI(twi, addr, &int_status.int_reg, MPU_6050_INT_STATUS);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	if(int_status.FIFO_OFLOW == 1) ret += 3;
	if(int_status.I2C_MST == 1) ret += 4;
	if(int_status.DATA_RDY == 1) ret += 5;
	
	return ret;
}

/*! \brief  Read external sensor value from the MPU6050
 *
 *	\warning External sensors for the MPU6050 are not supported by this library!
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	reg		byte to select which external sensor register needs to be read
 *	\param	*data	pointer to store the external sensor value
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t ext_sens_value_mpu6050(TWI_t *twi, uint8_t addr, uint8_t reg, uint8_t *data){
		uint8_t err, sensor_value;
		
		err = read_8bit_register_TWI(twi, addr, &sensor_value, reg);
		if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
				
		return 0;
}

/*! \brief  Disables temperature measurement
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t disable_temp_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t old_reg, err;
	
	err = read_8bit_register_TWI(twi, addr, &old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	old_reg = (1 << 3); //!< Disables Temperature measurements
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Enables temperature measurement
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t enable_temp_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t old_reg, err;
	
	err = read_8bit_register_TWI(twi, addr, &old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	old_reg = (0 << 3); //!< Enables Temperature measurements
	err = write_8bit_register_TWI(twi, addr, old_reg, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Resets the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 *
 *	When set to 1, this bit resets all internal registers to their default values.
 *	The bit automatically clears to 0 once the reset is done.
 */
uint8_t reset_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err;
	uint8_t reset = 0;
	
	reset = (1 << 7);
	
	err = write_8bit_register_TWI(twi, addr, reset, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Resets the accelerometer of the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 *
 *	This register is used to reset the analog and digital signal paths of the accelerometer.
 *	The reset will revert the signal path analog to digital converters and filters to their power up
 *	configurations
 *
 *	\note This function does not clear the sensor register
 */
uint8_t reset_accel_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err;
	uint8_t reset = 0;
	
	reset = (1 << 1);
	
	err = write_8bit_register_TWI(twi, addr, reset, MPU_6050_SIGNAL_PATH_RESET);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Resets the gyroscope of the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 *
 *	This register is used to reset the analog and digital signal paths of the gyroscope.
 *	The reset will revert the signal path analog to digital converters and filters to their power up
 *	configurations
 *
 *	\note This function does not clear the sensor register
 */
uint8_t reset_gyro_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err;
	uint8_t reset = 0;
	
	reset = (1 << 2);
	
	err = write_8bit_register_TWI(twi, addr, reset, MPU_6050_SIGNAL_PATH_RESET);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Resets the temperature sensor of the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 *
 *	This register is used to reset the analog and digital signal paths of the temperature sensors.
 *	The reset will revert the signal path analog to digital converters and filters to their power up
 *	configurations
 *
 *	\note This function does not clear the sensor register
 */
uint8_t reset_temp_mpu6050(TWI_t *twi, uint8_t addr){
	uint8_t err;
	uint8_t reset = 0;
	
	reset = (1 << 0);
	
	err = write_8bit_register_TWI(twi, addr, reset, MPU_6050_SIGNAL_PATH_RESET);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Select the clock source of the MPU6050
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	clk_sel	used to select what clk source the MPU6050 uses.
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t clk_sel_mpu6050(TWI_t *twi, uint8_t addr, uint8_t clk_sel){
	uint8_t err;
	MPU6050_PWR_MGMT_1_TYPE reg;
	
	err = read_8bit_register_TWI(twi, addr, &reg.PWR_MGMT_1, MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	reg.CLKSEL = clk_sel;
	
	err = write_8bit_register_TWI(twi, addr, reg.PWR_MGMT_1 , MPU_6050_PWR_MGMT_1);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;		
}


/*! \brief  Used for self testing
 *
 *	\warning	This function is not implemented!
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return	returns always 0
 */
uint8_t self_test_x_mpu6050(TWI_t *twi, uint8_t addr){
	return 0;
}

/*! \brief  Used for self testing
 *
 *	\warning	This function is not implemented!
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return returns always 0
 */
uint8_t self_test_y_mpu6050(TWI_t *twi, uint8_t addr){
	return 0;
}

/*! \brief  Used for self testing
 *
 *	\warning	This function is not implemented!
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *
 *  \return returns always 0
 */
uint8_t self_test_z_mpu6050(TWI_t *twi, uint8_t addr){
	return 0;
}

/*! \brief  Used for self testing
 *
 *	\warning	This function is not implemented!
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	xyz		select XA/YA/ZA_TEST
 *
 *  \return returns always 0
 */
uint8_t self_test_a_mpu6050(TWI_t *twi, uint8_t addr, uint8_t xyz){
	return 0;
}

/*! \brief  Set accelerometer scale/range
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	scale	select The scale
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t accel_set_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t scale){
	uint8_t err, data;
	MPU6050_ACCEL_CONFIG_TYPE ACCEL;
	ACCEL.ACCEL_CONFIG = 0;
	
	err = read_8bit_register_TWI(twi, addr, &data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	ACCEL.ACCEL_CONFIG = data;
	ACCEL.AFS_SEL = scale;
	data = ACCEL.ACCEL_CONFIG;
	
	err = write_8bit_register_TWI(twi, addr, data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;
}

/*! \brief  Get the accelerometer scale/range
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	*scale	pointer to store the scale/range value
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t accel_get_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t *scale){
	uint8_t err, data;
	MPU6050_ACCEL_CONFIG_TYPE ACCEL;
	
	err = read_8bit_register_TWI(twi, addr, &data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	ACCEL.ACCEL_CONFIG = data;
	(*scale) = ACCEL.AFS_SEL;
	
	return 0;	
}

/*! \brief  Set gyroscope scale/range
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	scale	select The scale
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t gyro_set_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t scale){
	uint8_t err, data;
	MPU6050_GYRO_CONFIG_TYPE GYRO;
	GYRO.GYRO_CONFIG = 0;
	
	err = read_8bit_register_TWI(twi, addr, &data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	GYRO.GYRO_CONFIG = data;
	GYRO.FS_SEL = scale;
	data = GYRO.GYRO_CONFIG;
	
	err = write_8bit_register_TWI(twi, addr, data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	return 0;	
}

/*! \brief  Get the gyroscope scale/range
 *
 *  \param  *twi	pointer to the TWI module that is connected to the MPU6050
 *	\param	addr	address of the MPU6050
 *	\param	*scale	pointer to store the scale/range value
 *
 *  \return 0 if succeeded if TWI/I2C bus is in use 1 otherwise returns 2
 */
uint8_t gyro_get_scale_mpu6050(TWI_t *twi, uint8_t addr, uint8_t *scale){
	uint8_t err, data;
	MPU6050_GYRO_CONFIG_TYPE GYRO;
	
	err = read_8bit_register_TWI(twi, addr, &data, MPU_6050_ACCEL_CONFIG);
	if(check_err_mpu6050(err) != 0) return check_err_mpu6050(err);
	
	GYRO.GYRO_CONFIG = data;	
	(*scale) = GYRO.FS_SEL;
	
	return 0;	
}

