/*
 * sensor.c
 *
 * Created: 15.10.2018 19:26:23
 *  Author: xeedn
 */ 
#include "sensor.h"
#include "i2c.h"
#include <math.h>

void calibrate(int16_t* arg_gyroBias, int16_t* arg_accelBias) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x80, 1); // Write a one to bit 7 reset bit; toggle reset device
	delay_ms(100);
	
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x01, 1);
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_2, 0x00, 1);
	delay_ms(200);
	
	// Configure device for bias calculation
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_ENABLE, 0x00, 1);   // Disable all interrupts
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_EN, 0x00, 1);      // Disable FIFO
	//sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_I2C_MST_CTRL, 0x00, 1); // Disable I2C master
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, 0x00, 1);    // Disable FIFO and I2C master modes
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, 0x0C, 1);    // Reset FIFO and DMP
	delay_ms(15);
	
	// Configure MPU6050 gyro and accelerometer for bias calculation
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_CONFIG, 0x01, 1);      // Set low-pass filter to 188 Hz
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_SMPLRT_DIV, 0x00, 1);  // Set sample rate to 1 kHz
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x00, 1);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00, 1); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
	//uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, 0x40, 1);   // Enable FIFO
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_EN, 0x78, 1);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	delay_ms(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_EN, 0x00, 1);        // Disable gyro and accelerometer sensors for FIFO
	receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_COUNTH, &data[0], 2); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_R_W, &data[0], 12); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
		
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}
	
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers; works well for gyro but not for accelerometer
	//  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
	//  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
	//  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
	//  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
	//  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
	//  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

	arg_gyroBias[0] = gyro_bias[0]; // construct gyro bias in deg/s for later manual subtraction
	arg_gyroBias[1] = gyro_bias[1];
	arg_gyroBias[2] = gyro_bias[2];

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_YA_OFFSET_H, &data[0], 2);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_ZA_OFFSET_H, &data[0], 2);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers; doesn't work well for accelerometer
	// Are we handling the temperature compensation bit correctly?
	//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
	//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
	//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
	//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
	//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
	//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

	// Output scaled accelerometer biases for manual subtraction in the main program
	arg_accelBias[0] = accel_bias[0];
	arg_accelBias[1] = accel_bias[1];
	arg_accelBias[2] = accel_bias[2];
}
void selfTest(float* destination) {
	uint8_t rawData[4];
	uint8_t selfTest[6];
	float factoryTrim[6];
	
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0xF0, 1);
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0xE0, 1);
	
	receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_SELF_TEST_X, &rawData, 4);
	
	selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
	selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
	selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
	 // Extract the gyration test results first
	 selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
	 selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
	 selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
	 // Process results to allow final comparison with factory set values
	 
	 factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
	 factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
	 factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
	 factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
	 factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
	 factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
	 
	 //  Output self-test results and factory trim calculation if desired
	 /*printf("Selftest: \n%d %d %d\n",selfTest[0], selfTest[1], selfTest[2]);
	 printf("%d %d %d\n",selfTest[3], selfTest[4], selfTest[5]);
	 printf("%d %d %d\n",factoryTrim[0], factoryTrim[1], factoryTrim[2]);
	 printf("%d %d %d\n",factoryTrim[3], factoryTrim[4], factoryTrim[5]);*/
	 
	  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	  // To get to percent, must multiply by 100 and subtract result from 100
	  for (int i = 0; i < 6; i++) {
		  destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
	  }
}

void setupSensor(Twi* interface) {
	printf("Sensor Setup...\n");
	sensor_interface = interface;
	delay_ms(200);
    openI2CServer(sensor_interface, TWI_CLK, MPU6050_I2C_ADDRESS);	
	printf("I2C Opened\n");
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x80, 1); // Write a one to bit 7 reset bit; toggle reset device
	printf("Device Reset\n");
	selfTest(SelfTest);
	
	printf("AccelSelfTest: %d %d %d\n",(int)(100*SelfTest[0]), (int)(100*SelfTest[1]), (int)(100*SelfTest[2]));
	printf("GyroSelfTest: %d %d %d\n",(int)(100*SelfTest[3]), (int)(100*SelfTest[4]), (int)(100*SelfTest[5]));
	
	calibrate(gyroBias, accelBias);
	
	printf("GyroBias: %d %d %d\n",gyroBias[0], gyroBias[1], gyroBias[2]);
	printf("AccelBias: %d %d %d\n",accelBias[0], accelBias[1], accelBias[2]);
	
	
	
	//Wake up MPU6050 and select gyro x clock
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, bit(MPU6050_CLKSEL_1), 1);
	
    //Enable FIFO
	//sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_EN, bit(MPU6050_XG_FIFO_EN) | bit(MPU6050_YG_FIFO_EN) | bit(MPU6050_ZG_FIFO_EN) | bit(MPU6050_ACCEL_FIFO_EN), 1);

    //Enable FIFO Interrupt
	//sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_ENABLE, MPU6050_FIFO_OFLOW_EN, 1);
	
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_ENABLE, bit(MPU6050_DATA_RDY_EN), 1);
	

    //Set interrupt mode (active low, open drain, keep low until clear, read on every clear)
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, bit(MPU6050_INT_LEVEL) | bit(MPU6050_INT_OPEN) | bit(MPU6050_INT_RD_CLEAR), 1);
	//uint8_t result;
	//receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, &result, 1);
	//printf("\nPin CFG register status: %#02x\n", result);
	//sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG,0, 1);
	
    //Set Digital low pass filter
	sendPacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_CONFIG, bit(MPU6050_DLPF_CFG2) | bit(MPU6050_DLPF_CFG1), 1);
	
    orientation_t orient = {
		.ax = 0,
		.ay = 0,
		.az = 0
	};
	
	current_orientation = orient;
	position_t position = {
		.x = 0,
		.y = 0,
		.z = 0
	};
	current_position = position;
	
	gyroSumX = gyroSumY = gyroSumZ = 0;
	accelSumX = accelSumY =	accelSumZ = 0;
	sumCounter = 0;
    //MPU6050_SMPLRT_DIV GYRO SAMPLE RATE DIVIDER
    //MPU6050_INT_STATUS
    //MPU6050_PWR_MGMT_1 CLOCK SELECTION?
	delay_ms(200);
	printf("Sensor Setup completed.\n");
}

uint8_t getSensorData(accel_t_gyro_union* accel_t_gyro) {
	//printf("Receiving Sensor Data.\n");
	
	if(!receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, (void*)(accel_t_gyro), 14)) {
		return 1;
	}
	
	SWAP (accel_t_gyro->reg.x_accel_h, accel_t_gyro->reg.x_accel_l);
	SWAP (accel_t_gyro->reg.y_accel_h, accel_t_gyro->reg.y_accel_l);
	SWAP (accel_t_gyro->reg.z_accel_h, accel_t_gyro->reg.z_accel_l);
	SWAP (accel_t_gyro->reg.t_h, accel_t_gyro->reg.t_l);
	SWAP (accel_t_gyro->reg.x_gyro_h, accel_t_gyro->reg.x_gyro_l);
	SWAP (accel_t_gyro->reg.y_gyro_h, accel_t_gyro->reg.y_gyro_l);
	SWAP (accel_t_gyro->reg.z_gyro_h, accel_t_gyro->reg.z_gyro_l);
	
	accel_t_gyro->value.x_accel -= accelBias[0];
	accel_t_gyro->value.y_accel -= accelBias[1];
	accel_t_gyro->value.z_accel -= accelBias[2];
	accel_t_gyro->value.x_gyro -= gyroBias[0];
	accel_t_gyro->value.y_gyro -= gyroBias[1];
	accel_t_gyro->value.z_gyro -= gyroBias[2];
#ifdef SWAP_X
    accel_t_gyro->value.x_accel *= -1;
	accel_t_gyro->value.x_gyro *= -1;
#endif
#ifdef SWAP_Y
	accel_t_gyro->value.y_accel *= -1;
	accel_t_gyro->value.y_gyro *= -1;
#endif
#ifdef SWAP_Z
	accel_t_gyro->value.z_accel *= -1;
	accel_t_gyro->value.z_gyro *= -1;
#endif

	
    return 0;
}

void getOrientation(orientation_t* orientation) {
	(*orientation) = current_orientation;
}

void getPosition(position_t* position) {
	(*position) = current_position;
}
uint32_t printCounter_sensor = SAMPLES_PER_SECOND;

uint8_t updateOrientation(void) {
	accel_t_gyro_union accel_t_gyro;
	orientation_t accelOrientation, gyroOrientation, dGyroOrientation;
	if(getSensorData(&accel_t_gyro)) {
		printf("Could not update orientation.\n");
		return 0;	
	}
	
	//Update orientation
	if(++sumCounter < 1000/SAMPLES_PER_SECOND) {
		gyroSumX += accel_t_gyro.value.x_gyro;
		gyroSumY += accel_t_gyro.value.y_gyro;
		gyroSumZ += accel_t_gyro.value.z_gyro;
		accelSumX += accel_t_gyro.value.x_accel;
		accelSumY += accel_t_gyro.value.y_accel;
		accelSumZ += accel_t_gyro.value.z_accel+(int)(G_1);
	} else {
		dGyroOrientation.ax = (float)(gyroSumX)/(DGS_250);
		dGyroOrientation.ay = (float)(gyroSumY)/(DGS_250);
		dGyroOrientation.az = (float)(gyroSumZ)/(DGS_250);
		
		gyroOrientation.ax = current_orientation.ax + dGyroOrientation.ax/(1000.0f);
		gyroOrientation.ay = current_orientation.ay + dGyroOrientation.ay/(1000.0f);
		gyroOrientation.az = current_orientation.az + dGyroOrientation.az/(1000.0f);
	
		float length, x,y,z, xy, zy, yx, zx;
		x = (float)accel_t_gyro.value.x_accel;
		y = (float)accel_t_gyro.value.y_accel;
		z = (float)accel_t_gyro.value.z_accel;
	
		//Calc xz-plane projection
		xy = x;
		zy = z;
		length = sqrt(xy*xy+zy*zy);
		xy /= length;
		zy /= length;
	
		//Calc yz-plane projection
		yx = y;
		zx = z;
		length = sqrt(yx*yx+zx*zx);
		yx /= length;
		zx /= length;
	
		//Apply angles between (0,0,-1)
		accelOrientation.ax = acos(-1.0f*zx)*(180.0f/3.14f);
		accelOrientation.ay = acos(-1.0f*zy)*(180.0f/3.14f);
		accelOrientation.az = 0;
		
		if(yx < 0) {
			accelOrientation.ax *= -1;
		}
		
		if(xy > 0) {
			accelOrientation.ay *= -1;
		}
		current_orientation.ax = (1.0f-COMPLEMENTARY_ALPHA)*(gyroOrientation.ax) + COMPLEMENTARY_ALPHA*accelOrientation.ax;
		current_orientation.ay = (1.0f-COMPLEMENTARY_ALPHA)*(gyroOrientation.ay) + COMPLEMENTARY_ALPHA*accelOrientation.ay;
		current_orientation.az = gyroOrientation.az;
		
		//Update position
		//current_position.x += (float)accelSumX*G_1_MPS/G_1/1000.0f;
		//current_position.y += (float)accelSumY*G_1_MPS/G_1/1000.0f;
		//current_position.z += (float)accelSumZ*G_1_MPS/G_1/1000.0f;
		
		
		if(--printCounter_sensor == 0) {
			printCounter_sensor = SAMPLES_PER_SECOND;
			/*printf("AcX = %d | AcY = %d | AcZ = %d | GyX = %d | GyY = %d | GyZ = %d\n",
			accel_t_gyro.value.x_accel,
			accel_t_gyro.value.y_accel,
			accel_t_gyro.value.z_accel,
			accel_t_gyro.value.x_gyro,
			accel_t_gyro.value.y_gyro,
			accel_t_gyro.value.z_gyro);*/
			//printf("dGyroOrientation: %d, %d, %d\n", (int) (1000.0f*dGyroOrientation.ax), (int) (1000.0f*dGyroOrientation.ay), (int) (1000.0f*dGyroOrientation.az));
			//printf("GyroOrientation: %d, %d, %d\n", (int) gyroOrientation.ax, (int) gyroOrientation.ay, (int) gyroOrientation.az);
			//printf("AccelOrientation: %d, %d, %d\n", (int) accelOrientation.ax, (int) accelOrientation.ay, (int) accelOrientation.az);
			//printf("New Orientation: %d, %d, %d\n", (int) current_orientation.ax, (int) current_orientation.ay, (int) current_orientation.az);
			//printf("AccelerationSum: %d, %d, %d\n", (int) accelSumX, (int) accelSumY, (int) accelSumZ);
		}
		gyroSumX = 0;
		gyroSumY = 0;
		gyroSumZ = 0;
		accelSumX = 0;
		accelSumY = 0;
		accelSumZ = 0;
		sumCounter = 0;
	}
	
	
	return 1;
}

uint32_t getFifoSensorData(accel_gyro_union* accel_gyro, uint32_t max_count) {
	
	uint16_t count;
	if(!receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_COUNTH, (void*)(&count), 2)) {
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
	
	if(!receivePacket(sensor_interface, MPU6050_I2C_ADDRESS, MPU6050_FIFO_R_W, (void*)(accel_gyro), count)) {
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

void sensorAxisTest(void) {;
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