#ifndef SENSOR_INO
#define SENSOR_INO

#include <asf.h>
#include "sensor_definitions.h"

#define SWAP_X
#define SWAP_Y
#define SWAP_Z

#define TWI_CLK 400000

#define bit(b) (1UL << (b))

struct SensorData {
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
};

uint8_t sendPacket(uint16_t address, const char data, int length) {
	twi_package_t packet_write = {
		.addr         = address,      // TWI slave memory address data
		.addr_length  = sizeof (uint16_t),    // TWI slave memory address data size
		.chip         = MPU6050_I2C_ADDRESS,      // TWI slave bus address
		.buffer       = (uint8_t *) &data, // transfer data source buffer
		.length       = length  // transfer data size (bytes)
	};
	
	printf("Sending Packet %#02x to %#02x\n", data, address);
	uint32_t err;
	if ((err = twi_master_write(TWI1, &packet_write)) != TWI_SUCCESS) {
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

uint32_t receivePacket(uint8_t address, void* data, uint32_t size) {
	twi_package_t packet_read = {
		.addr         = address,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = MPU6050_I2C_ADDRESS,      // TWI slave bus address
		.buffer       = data,        // transfer data destination buffer
		.length       = size                    // transfer data size (bytes)
	};
	
	
	uint32_t err;
	if ((err = twi_master_read(TWI1, &packet_read)) != TWI_SUCCESS) {
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


void openI2C() {
	//twi_options_t opt;
	//opt.master_clk = sysclk_get_peripheral_hz();
	//opt.speed      = TWI_CLK;
	twi_master_options_t opt = {
		.speed = TWI_CLK,
		.chip  = MPU6050_I2C_ADDRESS
	};
	twi_enable_master_mode(TWI1);
	if (twi_master_setup(TWI1, &opt) != TWI_SUCCESS) {
		printf("Could not initialize i2c.\n");
		return;
	}
	
	
	
	if (twi_probe(TWI1, MPU6050_I2C_ADDRESS) != TWI_SUCCESS) {
		printf("I2C Probe failed.\n");
	} else {
		printf("I2C Probe ok.\n");
	}
	
	uint8_t whoami;
	if(receivePacket(MPU6050_WHO_AM_I, &whoami, 1) > 0) {
		printf("Who Am I succeeded: %d\n", whoami);
	} else {
		printf("Who Am I failed.\n");
	}
}



void setupSensor() {
	delay_ms(200);
    openI2C();	
	
    //Enable FIFO
	//sendPacket(MPU6050_FIFO_EN, bit(MPU6050_XG_FIFO_EN) | bit(MPU6050_YG_FIFO_EN) | bit(MPU6050_ZG_FIFO_EN) | bit(MPU6050_ACCEL_FIFO_EN), 1);

    //Enable FIFO Interrupt
	//sendPacket(MPU6050_INT_ENABLE, MPU6050_FIFO_OFLOW_EN, 1);
	sendPacket(MPU6050_INT_ENABLE, bit(MPU6050_DATA_RDY_EN), 1);

    //Set interrupt mode (active low, open drain, keep low until clear, read on every clear)
	sendPacket(MPU6050_INT_PIN_CFG, bit(MPU6050_INT_LEVEL) | bit(MPU6050_INT_OPEN) | bit(MPU6050_LATCH_INT_EN) | bit(MPU6050_INT_RD_CLEAR), 1);
	//sendPacket(MPU6050_INT_PIN_CFG,0, 1);
	
    //Set Digital low pass filter
	sendPacket(MPU6050_CONFIG, bit(MPU6050_DLPF_CFG2) | bit(MPU6050_DLPF_CFG1), 1);
	
    //Wake up MPU6050 and select gyro x clock
	sendPacket(MPU6050_PWR_MGMT_1, bit(MPU6050_CLKSEL_1), 1);

    //MPU6050_SMPLRT_DIV GYRO SAMPLE RATE DIVIDER
    //MPU6050_INT_STATUS
    //MPU6050_PWR_MGMT_1 CLOCK SELECTION?
	
	printf("Sensor Setup completed.\n");
}

uint8_t getSensorData(accel_t_gyro_union* accel_t_gyro) {
	//printf("Receiving Sensor Data.\n");
	
	if(!receivePacket(MPU6050_ACCEL_XOUT_H, (void*)(accel_t_gyro), 14)) {
		return 1;
	}
	
	SWAP (accel_t_gyro->reg.x_accel_h, accel_t_gyro->reg.x_accel_l);
	SWAP (accel_t_gyro->reg.y_accel_h, accel_t_gyro->reg.y_accel_l);
	SWAP (accel_t_gyro->reg.z_accel_h, accel_t_gyro->reg.z_accel_l);
	SWAP (accel_t_gyro->reg.t_h, accel_t_gyro->reg.t_l);
	SWAP (accel_t_gyro->reg.x_gyro_h, accel_t_gyro->reg.x_gyro_l);
	SWAP (accel_t_gyro->reg.y_gyro_h, accel_t_gyro->reg.y_gyro_l);
	SWAP (accel_t_gyro->reg.z_gyro_h, accel_t_gyro->reg.z_gyro_l);
#ifdef SWAP_X
    accel_t_gyro->value.x_accel *= -1;
#endif
#ifdef SWAP_Y
	accel_t_gyro->value.y_accel *= -1;
#endif
#ifdef SWAP_Z
	accel_t_gyro->value.z_accel *= -1;
#endif
    return 0;
}

uint32_t getFifoSensorData(accel_gyro_union* accel_gyro, uint32_t max_count) {
	
	uint16_t count;
	if(!receivePacket(MPU6050_FIFO_COUNTH, (void*)(&count), 2)) {
		printf("Could not receive FIFO count or 0.\n");
		return 0;
	}
	//Reverse byteorder
	count = count >> 8 | count << 8;
	//Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    //Wire.write(MPU6050_FIFO_COUNTH);
    //Wire.endTransmission(false);
    //Wire.requestFrom(MPU6050_I2C_ADDRESS,2,false);
	//Wire.read()<<8|Wire.read();  // MPU6050_FIFO_COUNTH & MPU6050_FIFO_COUNTL)
    
	printf("Received %d sensor values.\n", count);
    if(count > max_count) {
		printf("Exceeded maximum count.\n");
		return 0;
	}
	
	if(!receivePacket(MPU6050_FIFO_R_W, (void*)(accel_gyro), count)) {
		printf("Could not receive FIFO data.\n");
		return 0;
	}
	
	for(int i=0;i<count/12;i++) {
		SWAP (accel_gyro[i].reg.x_accel_h, accel_gyro[i].reg.x_accel_l);
		SWAP (accel_gyro[i].reg.y_accel_h, accel_gyro[i].reg.y_accel_l);
		SWAP (accel_gyro[i].reg.z_accel_h, accel_gyro[i].reg.z_accel_l);
		SWAP (accel_gyro[i].reg.x_gyro_h, accel_gyro[i].reg.x_gyro_l);
		SWAP (accel_gyro[i].reg.y_gyro_h, accel_gyro[i].reg.y_gyro_l);
		SWAP (accel_gyro[i].reg.z_gyro_h, accel_gyro[i].reg.z_gyro_l);
		#ifdef SWAP_X
		accel_gyro[i].value.x_accel *= -1;
		#endif
		#ifdef SWAP_Y
		accel_gyro[i].value.y_accel *= -1;
		#endif
		#ifdef SWAP_Z
		accel_gyro[i].value.z_accel *= -1;
		#endif
	}
	/*Wire.write(MPU6050_FIFO_R_W);
    Wire.requestFrom(MPU_addr,count,true);*/
	
    /*if(count > 0) {
        SensorData* data = (*inData) = new SensorData[count/6];
        for(int i=0;i<count/6;i++) {
            data[i].AcX=Wire.read()<<8|Wire.read();
            data[i].AcY=Wire.read()<<8|Wire.read();
            data[i].AcZ=Wire.read()<<8|Wire.read();
            data[i].Tmp=0;
            data[i].GyX=Wire.read()<<8|Wire.read();
            data[i].GyY=Wire.read()<<8|Wire.read();
            data[i].GyZ=Wire.read()<<8|Wire.read();
            #ifdef SWAP_X
                data[i].AcX = -data[i].AcX;
            #endif
            #ifdef SWAP_Y
                data[i].AcY = -data[i].AcY;
            #endif
            #ifdef SWAP_Z
                data[i].AcZ = -data[i].AcZ;
            #endif
        }
    }*/
    return count/12;
}

#define OFF_THRESH 1000
#define LOW_THRESH 3000
#define HIGH_THRESH 5000

void sensorAxisTest() {;
    /*Serial.println("Running Sensor Axis Test. Move the sensor in all directions, while the direction is transmitted to the monitor. To finish type 'e'. Rdy?");
    pause();
    while (true) {
        SensorData data = getSensorData();
        if(data.AcX < -LOW_THRESH && data.AcY < -LOW_THRESH) {
            Serial.println("Back Left");
        } else if(data.AcX > LOW_THRESH && data.AcY < -LOW_THRESH) {
            Serial.println("Back Right");
        } else if(data.AcX < -LOW_THRESH && data.AcY > LOW_THRESH) {
            Serial.println("Front Left");
        } else if(data.AcX > LOW_THRESH && data.AcY > LOW_THRESH) {
            Serial.println("Front Right");
        } else if(data.AcX < -HIGH_THRESH && abs(data.AcY) < OFF_THRESH) {
            Serial.println("Left");
        } else if(data.AcX > HIGH_THRESH && abs(data.AcY) < OFF_THRESH) {
            Serial.println("Right");
        } else if(data.AcY < -HIGH_THRESH && abs(data.AcX) < OFF_THRESH) {
            Serial.println("Back");
        } else if(data.AcY > HIGH_THRESH && abs(data.AcX) < OFF_THRESH) {
            Serial.println("Front");
        } else {
            Serial.print("No Tilt: ");
            Serial.print("AcX = "); Serial.print(data.AcX);
            Serial.print(" | AcY = "); Serial.print(data.AcY);
            Serial.print(" | AcZ = "); Serial.print(data.AcZ);
            Serial.print(" | Tmp = "); Serial.print(data.Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
            Serial.print(" | GyX = "); Serial.print(data.GyX);
            Serial.print(" | GyY = "); Serial.print(data.GyY);
            Serial.println(" | GyZ = "); Serial.println(data.GyZ);
        }
        char currentChar = Serial.read();
        if (currentChar == 'e') break;
        delay(333);
    }
    pause();*/
}



#endif //SENSOR_INO