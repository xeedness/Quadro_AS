/*
 * i2c.h
 *
 * Created: 27.10.2018 10:48:09
 *  Author: xeedn
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <asf.h>

/** Memory size in bytes */
#define MEMORY_SIZE         512

/** The slave device instance*/
typedef struct _slave_device_t {
	/** PageAddress of the slave device*/
	uint16_t us_page_address;
	/** Offset of the memory access*/
	uint16_t us_offset_memory;
	/** Read address of the request*/
	uint8_t uc_acquire_address;
	/** Memory buffer*/
	uint8_t uc_memory[MEMORY_SIZE];
} slave_device_t;

/** The slave device instance*/
typedef struct receiver_state_t {
	uint8_t receiving;
	uint8_t current_size;
	uint8_t expected_size;
	uint8_t buffer[MEMORY_SIZE];
} receiver_state_t;

slave_device_t emulate_driver;
receiver_state_t receiver_state_twi0;
void (*twi0_data_received_callback)(uint8_t*, uint16_t);


void TWI_EnableIt(Twi *pTwi, uint32_t sources);
void TWI_DisableIt(Twi *pTwi, uint32_t sources);
uint8_t sendByte(Twi* twim, uint8_t slave_address, uint8_t address, uint8_t data);
uint8_t sendPacket(Twi* twim, uint8_t slave_address, uint8_t address, uint8_t* data, int length);
uint32_t receivePacket(Twi* twim, uint8_t slave_address, uint8_t address, void* data, uint32_t size);

uint8_t sendPacketTwi1(uint8_t slave_address, uint8_t address, uint32_t length, uint8_t* data);
uint32_t receivePacketTwi1(uint8_t slave_address, uint8_t address, uint32_t size, void* data);

void openI2CServer(Twi* twim, uint32_t clock, uint8_t address);
void openI2CClient(Twi* twim, uint8_t address, void (*recv_callback)(uint8_t*, uint16_t));

#endif /* I2C_H_ */