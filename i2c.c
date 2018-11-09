/*
 * i2c.c
 *
 * Created: 27.10.2018 10:48:18
 *  Author: xeedn
 */ 
#include "i2c.h"

void TWI_EnableIt(Twi *pTwi, uint32_t sources)
{
    pTwi->TWI_IER = sources;
}

void TWI_DisableIt(Twi *pTwi, uint32_t sources)
{
    pTwi->TWI_IDR = sources;
}

uint8_t sendPacket(Twi* twim, uint8_t slave_address, uint8_t address, uint8_t data, int length) {
	twi_package_t packet_write = {
		.addr         = {address},      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = slave_address,      // TWI slave bus address
		.buffer       = (uint8_t *) &data, // transfer data source buffer
		.length       = length  // transfer data size (bytes)
	};
	
	//printf("Sending Packet %#02x to %#02x\n", data, address);
	uint32_t err;
	if ((err = twi_master_write(twim, &packet_write)) != TWI_SUCCESS) {
		printf("Could not send packet: ");
		if(err == TWI_ERROR_TIMEOUT) {
			printf("TWI_ERROR_TIMEOUT\n");
			} else if(err == TWI_INVALID_ARGUMENT) {
			printf("TWI_INVALID_ARGUMENT\n");
			} else if(err == TWI_RECEIVE_NACK) {
			printf("TWI_RECEIVE_NACK\n");
			} else if(err == TWI_TIMEOUT) {
			printf("TWI_TIMEOUT\n");
			} else {
			printf("UNKNOWN\n");
		}
		return 0;
	}
	return 1;
}

uint32_t receivePacket(Twi* twim, uint8_t slave_address, uint8_t address, void* data, uint32_t size) {
	twi_package_t packet_read = {
		.addr         = {address},      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = slave_address,      // TWI slave bus address
		.buffer       = data,        // transfer data destination buffer
		.length       = size                    // transfer data size (bytes)
	};
	
	
	uint32_t err;
	if ((err = twi_master_read(twim, &packet_read)) != TWI_SUCCESS) {
		printf("Could not receive packet: ");
		if(err == TWI_ERROR_TIMEOUT) {
			printf("TWI_ERROR_TIMEOUT\n");
			} else if(err == TWI_INVALID_ARGUMENT) {
			printf("TWI_INVALID_ARGUMENT\n");
			} else if(err == TWI_RECEIVE_NACK) {
			printf("TWI_RECEIVE_NACK\n");
			} else if(err == TWI_TIMEOUT) {
			printf("TWI_TIMEOUT\n");
			} else {
			printf("UNKNOWN\n");
		}
		return 0;
	}
	return size;
}


void openI2CServer(Twi* twim, uint32_t clock, uint8_t address) {
	//twi_options_t opt;
	//opt.master_clk = sysclk_get_peripheral_hz();
	//opt.speed      = TWI_CLK;
	twi_master_options_t opt = {
		.speed = clock,
		.chip  = address
	};
	twi_enable_master_mode(twim);
	if (twi_master_setup(twim, &opt) != TWI_SUCCESS) {
		printf("Could not initialize i2c.\n");
		return;
	}
	
	if (twi_probe(twim, address) != TWI_SUCCESS) {
		printf("I2C Probe failed.\n");
		} else {
		printf("I2C Probe ok.\n");
	}
}

void openI2CClient(Twi* twim, uint8_t address, void (*recv_callback)(uint8_t*, uint16_t)) {
	delay_ms(2);
	/* Configure TWI interrupts */
	NVIC_DisableIRQ(TWI0_IRQn);
	NVIC_ClearPendingIRQ(TWI0_IRQn);
	NVIC_SetPriority(TWI0_IRQn, 0);
	NVIC_EnableIRQ(TWI0_IRQn);
	delay_ms(2);
	
	//twi_get_pdc_base(twim) = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	twim->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	
	delay_ms(2);
	twi0_data_received_callback = recv_callback;
	twi_slave_setup(twim, address);
	twi_read_byte(twim);
	delay_ms(2);
	
	receiver_state_twi0.current_size = 0;
	receiver_state_twi0.expected_size = 0;
	receiver_state_twi0.receiving = 0;
	twi_enable_interrupt(twim, TWI_IER_SVACC);
	delay_ms(2);
}

void TWI0_Handler(void) {
	uint32_t status;

	status = twi_get_interrupt_status(TWI0);

	if (receiver_state_twi0.receiving == 0 && 
	((status & TWI_SR_SVACC) == TWI_SR_SVACC)) {
		NVIC_DisableIRQ(PIOB_IRQn);
		TWI_DisableIt(TWI0, TWI_IDR_SVACC);
		TWI_EnableIt(TWI0, TWI_IER_RXRDY | TWI_IER_GACC
		| TWI_IER_NACK | TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);
		
		receiver_state_twi0.current_size = 0;
		receiver_state_twi0.expected_size = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if ((status & TWI_SR_SVREAD) == TWI_SR_SVREAD) {
			//printf("Write mode\n");
			receiver_state_twi0.receiving = 2;
			// create a default 1-byte response
			//write((uint8_t) 0);
		} else {
			//printf("Read mode\n");
			receiver_state_twi0.receiving = 1;
		}
	}

	if (receiver_state_twi0.receiving == 1) {
		if (((status & TWI_SR_RXRDY) == TWI_SR_RXRDY)) {
			//printf("Reading Byte\n");
			receiver_state_twi0.buffer[receiver_state_twi0.current_size++] = (twi_read_byte(TWI0) & 0xFF);
		}
	}

	/*if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
			c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}*/

	if (receiver_state_twi0.receiving != 0 && 
	((status & TWI_SR_EOSACC) == TWI_SR_EOSACC)) {
		if (receiver_state_twi0.receiving == 1) {
			(*twi0_data_received_callback)(receiver_state_twi0.buffer, receiver_state_twi0.current_size);
		}
		NVIC_EnableIRQ(PIOB_IRQn);

		// Transfer completed
		TWI_EnableIt(TWI0, TWI_IER_SVACC);
		TWI_DisableIt(TWI0, TWI_IDR_RXRDY | TWI_IDR_GACC
		| TWI_IDR_NACK | TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IDR_TXCOMP);
		receiver_state_twi0.receiving = 0;
	}
}