/*
 * mpu9250.c
 *
 * Created: 5/23/2021 12:46:13 PM
 *  Author: xeedn
 */ 
#include <asf.h>
#include "mpu9250.h"

#include "inv_mpu.h"
#include "MPU9250_RegisterMap.h"
#include "i2c.h"

#include <math.h>
#include <stdint.h>

float gyroSensFactor = 0;
unsigned short accelSensFactor = 0;
float magSensFactor = 6.665f;

#define TWI_CLK 390000
#define MPU9250_I2C_ADDRESS 0x68

typedef struct raw_vec {
	int x, y, z;
} raw_vec_t;

typedef struct real_vec {
	float x, y, z;
} real_vec_t;

raw_vec_t raw_accel = { 0,0,0 };
raw_vec_t raw_gyro = { 0,0,0 };
raw_vec_t raw_comp = { 0,0,0 };


bool dataReady(void);
int update(unsigned char sensors);
int updateGyro(void);
int updateAccel(void);
int updateCompass(void);
void scaleAccel(raw_vec_t* in, real_vec_t* out);
void scaleGyro(raw_vec_t* in, real_vec_t* out);
void scaleCompass(raw_vec_t* in, real_vec_t* out);
void printIMUData(void);

unsigned long time = 0;

int mpu9250_setup(void) {
	//Enable sensor interrupt
	pmc_enable_periph_clk(ID_PIOB);
	
	// Configure sensor vcc
	pio_set_output(PIOB, PIO_PB21, LOW, DISABLE, ENABLE);
	pio_clear(PIOB, PIO_PB21);
	
	// Ensure powering down sensor and turn back on after a second
	delay_s(1);
	pio_set(PIOB, PIO_PB21);
	
	delay_s(1);
	
	openI2CServer(TWI1, TWI_CLK, MPU9250_I2C_ADDRESS);
	
	int result;
	struct int_param_s int_param;
	result = mpu_init(&int_param);
	
	if(result) {
		printf("Init failed.\n");
		return result;
	}
	if(mpu_set_bypass(1)) {
		printf("Set Bypass failed.\n");
		return 1;
	}
	
	
	if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0) {
		printf("Set Sensors failed.\n");
	}
	
	if(mpu_get_gyro_sens(&gyroSensFactor) != 0) {
		printf("Gyro Sensitivity failed.\n");
	}
	
	if(mpu_get_accel_sens(&accelSensFactor) != 0) {
		printf("Accel Sensitivity failed.\n");
	}

	mpu_set_sample_rate(10);
	
	mpu_set_compass_sample_rate(10);
	
	
	/*
	//Enable sensor interrupt
	pmc_enable_periph_clk(ID_PIOB);
	// Configure sensor interrupt
	pio_set_input(PIOB, PIO_PB26, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB26, PIO_IT_FALL_EDGE, onSensorDataReady);
	pio_enable_interrupt(PIOB, PIO_PB26);
	NVIC_SetPriority(PIOB_IRQn, 2);
	NVIC_EnableIRQ(PIOB_IRQn);
	*/
	
	return 0;
}

void mpu9250_loop(void) {
	
	while(1) {
		delay_ms(10);
		// dataReady() checks to see if new accel/gyro data
		// is available. It will return a boolean true or false
		// (New magnetometer data cannot be checked, as the library
		//  runs that sensor in single-conversion mode.)
		if ( dataReady() )
		{
			// Call update() to update the imu objects sensor data.
			// You can specify which sensors to update by combining
			// UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
			// UPDATE_TEMPERATURE.
			// (The update function defaults to accel, gyro, compass,
			//  so you don't have to specify these values.)
			update(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);
			printIMUData();
		}
	}
}

bool dataReady(void) {
	unsigned char intStatusReg;
	
	if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == 0)
	{
		return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
	}
	return false;
}

int update(unsigned char sensors) {
	int aErr = 0;
	int gErr = 0;
	int mErr = 0;
	int tErr = 0;
	
	if (sensors & INV_XYZ_ACCEL) {
		aErr = updateAccel();
	}
	if (sensors & INV_XYZ_GYRO) {
		gErr = updateGyro();
	}
	if (sensors & INV_XYZ_COMPASS) {
		mErr = updateCompass();
	}
	/*if (sensors & UPDATE_TEMP)
	tErr = updateTemperature();*/
	
	return aErr | gErr | mErr | tErr;
}

int updateGyro(void) {
	short data[3];
	
	if (mpu_get_gyro_reg(data, &time))
	{
		return 1;
	}
	raw_gyro.x = data[0];
	raw_gyro.y = data[1];
	raw_gyro.z = data[2];
	return 0;
}

int updateAccel(void) {
	short data[3];
	
	if (mpu_get_accel_reg(data, &time))
	{
		return 1;
	}
	raw_accel.x = data[0];
	raw_accel.y = data[1];
	raw_accel.z = data[2];
	return 0;
}

int updateCompass(void) {
	short data[3];
	
	if (mpu_get_compass_reg(data, &time))
	{
		return 1;
	}
	raw_comp.x = data[0];
	raw_comp.y = data[1];
	raw_comp.z = data[2];
	return 0;
}

void scaleGyro(raw_vec_t* in, real_vec_t* out) {
	out->x = (float) in->x / (float) gyroSensFactor;
	out->y = (float) in->y / (float) gyroSensFactor;
	out->z = (float) in->z / (float) gyroSensFactor;
}

void scaleAccel(raw_vec_t* in, real_vec_t* out) {
	out->x = (float) in->x / (float) accelSensFactor;
	out->y = (float) in->y / (float) accelSensFactor;
	out->z = (float) in->z / (float) accelSensFactor;
}

void scaleCompass(raw_vec_t* in, real_vec_t* out) {
	out->x = (float) in->x / (float) magSensFactor;
	out->y = (float) in->y / (float) magSensFactor;
	out->z = (float) in->z / (float) magSensFactor;
}

void printIMUData(void) {
	real_vec_t gyro;
	real_vec_t accel;
	real_vec_t mag;
	
	scaleGyro(&raw_gyro, &gyro);
	scaleAccel(&raw_accel, &accel);
	scaleCompass(&raw_comp, &mag);
		 
	printf("Gyro: %d %d %d\n", raw_gyro.x, raw_gyro.y, raw_gyro.z);
	
	float magMag = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
	
	printf("Gyro: %.4f %.4f %.4f\n", gyro.x, gyro.y, gyro.z);
	printf("Accel: %.2f %.2f %.2f\n", accel.x, accel.y, accel.z);
	printf("Mag: %.2f %.2f %.2f: %.2f\n", mag.x, mag.y, mag.z, magMag);
	
}