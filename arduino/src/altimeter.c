/*
 * altimeter.c
 *
 * Created: 25.09.2020 17:40:08
 *  Author: xeedn
 */ 

#include "altimeter.h"
#include "bmp3.h"
#include "i2c.h"
#include <math.h>
#include "timer.h"

struct bmp3_dev	device;

#define TWI_CLK 50000

uint8_t dataReady = 0;
uint32_t altitute_tick;
double altitude = 0;
double basePressure = 0;
double Ralt = 8.314;
double M = 0.029;
double galt = 9.81;

void onAltimeterReady(uint32_t arg0, uint32_t arg1);
BMP3_INTF_RET_TYPE altimeter_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
BMP3_INTF_RET_TYPE altimeter_i2c_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr);
void altimeter_delay(uint32_t period, void *intf_ptr);

void setupAltimeter(Twi* interface) {
	// Configure sensor vcc
	pio_set_output(PIOA, PIO_PA3, LOW, DISABLE, ENABLE);
	pio_clear(PIOA, PIO_PA3);
	// Ensure powering down sensor and turn back on after a second
	delay_s(1);
	pio_set(PIOA, PIO_PA3);
	delay_s(1);
	
	// Enable sensor interrupt
	pmc_enable_periph_clk(ID_PIOA);
	
	// Configure sensor interrupt
	pio_set_input(PIOA, PIO_PA2, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA2, PIO_IT_FALL_EDGE, onAltimeterReady);
	pio_enable_interrupt(PIOA, PIO_PA2);
	NVIC_SetPriority(PIOA_IRQn, 2);
	NVIC_EnableIRQ(PIOA_IRQn);
	delay_ms(500);
	openI2CServer(interface, TWI_CLK, BMP3_ADDR_I2C_SEC);
	device.intf = BMP3_I2C_INTF;
	device.intf_ptr = interface;
	device.read = altimeter_i2c_read;
	device.write = altimeter_i2c_write;
	device.delay_us = altimeter_delay;
	device.settings.op_mode = BMP3_MODE_NORMAL;
	device.settings.press_en = BMP3_ENABLE;
	device.settings.temp_en = BMP3_ENABLE;
	// Filter
	device.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
	device.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	device.settings.odr_filter.odr = BMP3_ODR_50_HZ;
	device.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
	// Interrupt
	device.settings.int_settings.drdy_en = BMP3_ENABLE;
	device.settings.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
	device.settings.int_settings.level = BMP3_INT_PIN_ACTIVE_LOW;
	device.settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
	
	if(bmp3_init(&device)) {
		printf("Some problem occurred during bmp3_init.\n");
	}
	
	if(bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER | BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH |BMP3_SEL_DRDY_EN, &device)) {
		printf("Some problem occurred during bmp3_set_op_mode.\n");
	}
	
	if(bmp3_set_op_mode(&device)) {
		printf("Some problem occurred during bmp3_set_op_mode.\n");
	}	

	delay_ms(200);
	
	struct bmp3_data data;
	
	int samples = 0;
	while(samples < 100) {
		if(bmp3_get_sensor_data(BMP3_ALL, &data, &device)) {
			printf("An error occured during altimeter data reading.\n");
		}
	
		basePressure += (double)(data.pressure)/100;
		samples++;
	}
	
	basePressure /= samples;
	printf("Base Pressure %.2f.\n", basePressure);
	// Set initial value of sensor data ready flag
	
	delay_ms(200);
}

uint8_t updateAltitude(void) {
	if(dataReady) {
		dataReady = 0;
		struct bmp3_data data;
		if(bmp3_get_sensor_data(BMP3_ALL, &data, &device)) {
			printf("An error occured during altimeter data reading.\n");
			return 1;
		}
	
		double currentPressure = (double)(data.pressure)/100;
		double T = 273.15 + (double)(data.temperature)/100;
		
		altitude = -log(currentPressure/basePressure)*Ralt*T/(M*galt);
		altitute_tick = current_ticks();
		//printf("Pressure: %.2f, Temp: %.2f, Altitude: %.2f\n", currentPressure, T, altitude);
		return 0;
	} else {
		return 1;
	}
}

double getAltitude(void) {
	return altitude;
}

uint32_t getAltitudeTick(void) {
	return altitute_tick;
}

BMP3_INTF_RET_TYPE altimeter_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr) {
	if(receivePacket((Twi*)intf_ptr, BMP3_ADDR_I2C_SEC, reg_addr, read_data, len)) {
		return BMP3_INTF_RET_SUCCESS;
	} else {
		return BMP3_ERR_FATAL;
	}
}

BMP3_INTF_RET_TYPE altimeter_i2c_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr) {
	if(sendPacket((Twi*)intf_ptr, BMP3_ADDR_I2C_SEC, reg_addr, (uint8_t*)read_data, len)) {
		return BMP3_INTF_RET_SUCCESS;
	} else {
		return BMP3_ERR_FATAL;
	}
}

void altimeter_delay(uint32_t period, void *intf_ptr) {
	delay_us(period);
}

void onAltimeterReady(uint32_t arg0, uint32_t arg1) {
	dataReady = 1;
	updateAltitude();
}