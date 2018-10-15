#ifndef SENSOR_INO
#define SENSOR_INO

#include <asf.h>
#include "sensor_definitions.h"

#define SWAP_X
#define SWAP_Y
#define SWAP_Z

#define SAMPLES_PER_SECOND 100
#define COMPLEMENTARY_ALPHA	0.1f	

#define TWI_CLK 400000

#define G_1 16384.0f
#define DGS_250 131.072f

#define OFF_THRESH 1000
#define LOW_THRESH 3000
#define HIGH_THRESH 5000

#define bit(b) (1UL << (b))

int16_t gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
float SelfTest[6];               // Gyro and accelerometer self-test sensor output

struct SensorData {
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
};

uint32_t sumCounter;
int32_t gyroSumX, gyroSumY, gyroSumZ;

typedef struct orientation {
	float ax, ay, az;
} orientation_t;

orientation_t current_orientation;

uint8_t sendPacket(uint8_t address, uint8_t data, int length);
uint32_t receivePacket(uint8_t address, void* data, uint32_t size);
void openI2C(void);
void calibrate(int16_t* arg_gyroBias, int16_t* arg_accelBias);
void selfTest(float* destination);
void setupSensor(void);
uint8_t getSensorData(accel_t_gyro_union* accel_t_gyro);
void getOrientation(orientation_t* orientation);
uint8_t updateOrientation(void);
uint32_t getFifoSensorData(accel_gyro_union* accel_gyro, uint32_t max_count);
void sensorAxisTest(void);



#endif //SENSOR_INO