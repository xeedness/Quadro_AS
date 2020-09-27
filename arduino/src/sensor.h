#ifndef SENSOR_INO
#define SENSOR_INO

#include <asf.h>
#include "sensor_definitions.h"

//#define SWAP_X
//#define SWAP_Y
//#define SWAP_Z

#define SAMPLES_PER_SECOND 100

#define TWI_CLK 390000

#define G_1 16384.0f
#define DGS_250 131.072f
#define G_1_MPS 9.80665f

#define OFF_THRESH 1000
#define LOW_THRESH 3000
#define HIGH_THRESH 5000

#define DEG_TO_RAD_FACTOR 0.01745f //PI / 180

#define bit(b) (1UL << (b))

Twi* sensor_interface;

int16_t gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
float SelfTest[6];               // Gyro and accelerometer self-test sensor output

struct SensorData {
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
};

uint32_t sumCounter;
int32_t gyroSumX, gyroSumY, gyroSumZ;
int32_t accelSumX, accelSumY, accelSumZ;

typedef struct angular_rate {
	float wx, wy, wz;	
} angular_rate_t;

typedef struct orientation {
	float ax, ay, az;
} orientation_t;

typedef struct position{
	float x, y, z;
} position_t;

typedef struct drone_speed {
	float vx, vy, vz;	
} drone_speed_t;

typedef struct acceleration {
	float ax, ay, az;
} acceleration_t;

uint32_t last_sensor_tick;


void calibrate(int16_t* arg_gyroBias, int16_t* arg_accelBias);
void selfTest(float* destination);
void setupSensor(Twi* interface, uint32_t currentTicks);
uint8_t updateSensorData(void);

void getAngleOffsets(float* x_offset, float* y_offset);
void getRawAcceleration(float* ax, float* ay, float* az);
void getAnglesOfRawAcceleration(float* x_dst, float* y_dst);
void getRawValuesGyro(float* x_dst, float* y_dst, float* z_dst);
float getG1(void);
uint32_t getSensorTick(void);

uint8_t updateOrientation(void);
uint32_t getFifoSensorData(accel_gyro_union* accel_gyro, uint32_t max_count);
void sensorAxisTest(void);
void onSensorDataReady(uint32_t arg0, uint32_t arg1);
uint8_t is_sensor_alive(void);

#endif //SENSOR_INO