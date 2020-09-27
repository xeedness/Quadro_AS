/*
 * kalman.c
 *
 * Created: 24.09.2020 16:34:56
 *  Author: xeedn
 */ 

#include "kalman.h"
#include "math_helpers.h"
#include "timer.h"
#include "config.h"
#include "call_counter.h"
#include <math.h>
#include "altimeter.h"

uint32_t last_kalman_tick;
uint32_t last_altitude_tick;
float current_altitude = 0;
float current_vertical_speed = 0;
float current_vertical_acceleration = 0;

typedef struct kalman_orientation {
	float X[4];
	float Xp[4];
	float U[2];
	float P[4][4];
	float R[4][4];
	float Q[4][4];
	float A[4][4];
	float B[4][2];
	float I[4][4];
} kalman_orientation_t;

kalman_orientation_t k1;

typedef struct filter_altitude {
	float X[3];	
} filter_altitude_t;

filter_altitude_t k2;

void updateOrientationEstimate(float dt);
void updateKalmanOrientationInternal(orientation_t measured_phi, angular_rate_t measured_omega);
float getVerticalAcceleration(void);

void setupKalman(void) {
	// Startup state is at zero angle and at zero angular velocity
	k1.X[0] = 0; k1.X[1] = 0; k1.X[2] = 0; k1.X[3] = 0;
	
	// Set control variables to zero, since we are currently not feeding them back from PID
	k1.U[0] = 0; k1.U[0] = 0;
	
	// State Covariance: Set to zero, because we don't know better
	k1.P[0][0] = 0; k1.P[0][1] = 0; k1.P[0][2] = 0; k1.P[0][3] = 0;
	k1.P[1][0] = 0; k1.P[1][1] = 0; k1.P[1][2] = 0; k1.P[1][3] = 0;
	k1.P[2][0] = 0; k1.P[2][1] = 0; k1.P[2][2] = 0; k1.P[2][3] = 0;
	k1.P[3][0] = 0; k1.P[3][1] = 0; k1.P[3][2] = 0; k1.P[3][3] = 0;
	
	// Measurement covariance: Only variances, no covariances, because we don't know
	k1.R[0][0] = 0.1; k1.R[0][1] = 0; k1.R[0][2] = 0; k1.R[0][3] = 0;
	k1.R[1][0] = 0; k1.R[1][1] = 0.1; k1.R[1][2] = 0; k1.R[1][3] = 0;
	k1.R[2][0] = 0; k1.R[2][1] = 0; k1.R[2][2] = 0.001; k1.R[2][3] = 0;
	k1.R[3][0] = 0; k1.R[3][1] = 0; k1.R[3][2] = 0; k1.R[3][3] = 0.0004;
	
	// Process Covariance: Only variances, no covariances
	k1.Q[0][0] = 0.001; k1.Q[0][1] = 0; k1.Q[0][2] = 0; k1.Q[0][3] = 0;
	k1.Q[1][0] = 0; k1.Q[1][1] = 0.001; k1.Q[1][2] = 0; k1.Q[1][3] = 0;
	k1.Q[2][0] = 0; k1.Q[2][1] = 0; k1.Q[2][2] = 0.1; k1.Q[2][3] = 0;
	k1.Q[3][0] = 0; k1.Q[3][1] = 0; k1.Q[3][2] = 0; k1.Q[3][3] = 0.1;
	
	// Identity for subtraction
	k1.I[0][0] = 1; k1.I[0][1] = 0; k1.I[0][2] = 0; k1.I[0][3] = 0;
	k1.I[1][0] = 0; k1.I[1][1] = 1; k1.I[1][2] = 0; k1.I[1][3] = 0;
	k1.I[2][0] = 0; k1.I[2][1] = 0; k1.I[2][2] = 1; k1.I[2][3] = 0;
	k1.I[3][0] = 0; k1.I[3][1] = 0; k1.I[3][2] = 0; k1.I[3][3] = 1;
	
	last_altitude_tick = last_kalman_tick = current_ticks();
	
	printf("Kalman Setup done\n");
}

void getKalmanOrientationEstimate(orientation_t* phi, angular_rate_t* omega) {
	float dt = elapsed_time_s(last_kalman_tick);
	updateOrientationEstimate(dt);
	
	phi->ax = k1.Xp[0];
	phi->ay = k1.Xp[1];
	phi->az = 0;
	omega->wx = k1.Xp[2];
	omega->wy = k1.Xp[3];
	omega->wz = 0;
}
void updateKalmanOrientation(void) {		
	if(sensor_config.enabled) {
		orientation_t measured_phi;
		angular_rate_t measured_omega;
		getAnglesOfRawAcceleration(&measured_phi.ax, &measured_phi.ay);
		getRawValuesGyro(&measured_omega.wx, &measured_omega.wy, &measured_omega.wz);
		
		updateKalmanOrientationInternal(measured_phi, measured_omega);
	}
}

void updateKalmanOrientationInternal(orientation_t measured_phi, angular_rate_t measured_omega) {	
	
	sensor_called();
	float S[4][4], Sinv[4][4], K[4][4], Y[4], Pp[4][4];
	Y[0] = measured_phi.ax;
	Y[1] = measured_phi.ay;
	Y[2] = measured_omega.wx;
	Y[3] = measured_omega.wy;
	
	//printf("\nY:");
	//printf("\n%.7f %.7f %.7f %.7f", k1.Y[0], k1.Y[1], k1.Y[2], k1.Y[3]);
	
	float dt = elapsed_time_s(last_kalman_tick);
	last_kalman_tick = current_ticks();
	
	
	updateOrientationEstimate(dt);
	
	// Update Process Covariance Estimate
	// P = APAt + Q:
	float At[4][4], PAt[4][4], APAt[4][4];
	transpose(At, k1.A, 4);
	multiply(PAt, k1.P, At, 4, 4, 4, 4);
	multiply(APAt, k1.A, PAt, 4, 4, 4, 4);
	add(Pp, APAt, k1.Q, 4, 4);
	
	
	// Calculate Kalman Gain
	// S = HPHt + R:
	// H is identity. 
	// TODO Check if that's always true
	add(S, Pp, k1.R, 4, 4);
	
	// K = PHt * inv(S):
	if(invert(Sinv, S, 4)) {
		return;
	}
	
	multiply(K, Pp, Sinv, 4, 4, 4, 4);
	
	// Calculate new state based on kalman gain
	// X = Xp + K * (Y-Xp)
	float YmXp[4], KmYmXp[4];
	subtract(YmXp, Y, k1.Xp, 4, 1);
	multiply(KmYmXp, K, YmXp, 4, 4, 4, 1);
	add(k1.X, k1.Xp, KmYmXp, 4, 1);
	
	// Calculate new process covariance based on kalman gain
	// P = (I-KH) * Pp
	float ImK[4][4];
	subtract(ImK, k1.I, K, 4, 4);
	multiply(k1.P, ImK, Pp, 4, 4, 4, 4);
	
	/*printf("\nP:");
	printf("\n%.7f %.7f %.7f %.7f", k1.P[0][0], k1.P[0][1], k1.P[0][2], k1.P[0][3]);
	printf("\n%.7f %.7f %.7f %.7f", k1.P[1][0], k1.P[1][1], k1.P[1][2], k1.P[1][3]);
	printf("\n%.7f %.7f %.7f %.7f", k1.P[2][0], k1.P[2][1], k1.P[2][2], k1.P[2][3]);
	printf("\n%.7f %.7f %.7f %.7f\n", k1.P[3][0], k1.P[3][1], k1.P[3][2], k1.P[3][3]);*/
	
	//printf("\nK:");
	//printf("\n%.7f %.7f %.7f %.7f", K[0][0], K[0][1], K[0][2], K[0][3]);
	//printf("\n%.7f %.7f %.7f %.7f", K[1][0], K[1][1], K[1][2], K[1][3]);
	//printf("\n%.7f %.7f %.7f %.7f", K[2][0], K[2][1], K[2][2], K[2][3]);
	//printf("\n%.7f %.7f %.7f %.7f\n", K[3][0], K[3][1], K[3][2], K[3][3]);
	//
	//
	//printf("\nS:");
	//printf("\n%.7f %.7f %.7f %.7f", S[0][0], S[0][1], S[0][2], S[0][3]);
	//printf("\n%.7f %.7f %.7f %.7f", S[1][0], S[1][1], S[1][2], S[1][3]);
	//printf("\n%.7f %.7f %.7f %.7f", S[2][0], S[2][1], S[2][2], S[2][3]);
	//printf("\n%.7f %.7f %.7f %.7f\n", S[3][0], S[3][1], S[3][2], S[3][3]);
	//
	//printf("\nSinv:");
	//printf("\n%.7f %.7f %.7f %.7f", Sinv[0][0], Sinv[0][1], Sinv[0][2], Sinv[0][3]);
	//printf("\n%.7f %.7f %.7f %.7f", Sinv[1][0], Sinv[1][1], Sinv[1][2], Sinv[1][3]);
	//printf("\n%.7f %.7f %.7f %.7f", Sinv[2][0], Sinv[2][1], Sinv[2][2], Sinv[2][3]);
	//printf("\n%.7f %.7f %.7f %.7f\n", Sinv[3][0], Sinv[3][1], Sinv[3][2], Sinv[3][3]);
}

void updateOrientationEstimate(float dt) {
	float dt2 = dt*dt;
	
	// Set A and u matrix using dt
	k1.A[0][0] = 1; k1.A[0][1] = 0; k1.A[0][2] = dt; k1.A[0][3] = 0;
	k1.A[1][0] = 0; k1.A[1][1] = 1; k1.A[1][2] = 0; k1.A[1][3] = dt;
	k1.A[2][0] = 0; k1.A[2][1] = 0; k1.A[2][2] = 1; k1.A[2][3] = 0;
	k1.A[3][0] = 0; k1.A[3][1] = 0; k1.A[3][2] = 0; k1.A[3][3] = 1;
	
	k1.B[0][0] = 0.5f*dt2; k1.B[0][1] = 0;
	k1.B[1][0] = 0; k1.B[1][1] = 0.5f*dt2;
	k1.B[2][0] = dt; k1.B[2][1] = 0;
	k1.B[3][0] = 0; k1.B[3][1] = dt;
	
	// Xp = AX + BU:
	float AX[4];
	float BU[4];
	multiply(AX, k1.A, k1.X, 4, 4, 4, 1);
	multiply(BU, k1.B, k1.U, 4, 2, 2, 1);
	add(k1.Xp, AX, BU, 4, 1);
	
	// printf("\nP:");
	// printf("\n%.7f %.7f %.7f %.7f", k1.P[0], k1.P[1], k1.P[2], k1.P[3]);
}


void getKalmanAltituteEstimate(altimeter_t* altimeter_data) {	
	altimeter_data->altitude = current_altitude;
	altimeter_data->vertical_velocity = current_vertical_speed;
	altimeter_data->vertical_acceleration = current_vertical_acceleration;
}

int counter = 0;
void updateKalmanAltitude(void) {
	float dt = elapsed_time_s(last_kalman_tick);
	updateOrientationEstimate(dt);
	last_kalman_tick = current_ticks();
	current_vertical_acceleration = getVerticalAcceleration();
	float mAltitude = (float)getAltitude();
	float altitude_gain = -1.1547f; //-0.2828f; //-0.6325f;//-0.1925f;
	float speed_gain = -0.6667; //-0.2f;//-0.0185f;
	float dAltitude = (current_altitude - mAltitude);
	current_altitude += dt * current_vertical_speed;
	current_altitude += dAltitude * dt * altitude_gain + dAltitude * speed_gain * dt * dt * 0.5f;
	current_altitude += dt * dt * 0.5f * current_vertical_acceleration;
	current_vertical_speed += speed_gain * dt * dAltitude;
	current_vertical_speed += dt * current_vertical_acceleration;
	
	//printf("Am: %0.3f, A: %.3f, dA: %.3f, d2A: %.3f\n", mAltitude, current_altitude, current_vertical_speed, current_vertical_acceleration);
	//printf("%0.3f %.3f\n", mAltitude, current_vertical_acceleration);
}

float getVerticalAcceleration(void) {
	orientation_t attitute;
	angular_rate_t rates;
	getKalmanOrientationEstimate(&attitute, &rates);	
	float ax, ay, az;
	getRawAcceleration(&ax, &ay, &az);
	float x_angle_offset, y_angle_offset;
	getAngleOffsets(&x_angle_offset, &y_angle_offset);
	// Adjust acceleration by rotated g force / Derived by Rx * Ry * GBase (offset or 0,0,G)
	float sinx = sin(attitute.ax + x_angle_offset);
	float cosx = cos(attitute.ax + x_angle_offset);
	float siny = sin(attitute.ay + y_angle_offset);
	float cosy = cos(attitute.ay + y_angle_offset);
	//float gx = G_1 * siny;
	//float gy = -sinx * cosy * G_1;
	//float gz = cosx * cosy * G_1;
	/*float gx = cosy * accelBias[0] + siny * accelBias[2];
	float gy = cosx * 1 * accelBias[1] + -sinx * (-siny * accelBias[0] + cosy * accelBias[2]);
	float gz = sinx * (1 * accelBias[1]) + cosx * (-siny * accelBias[0] + cosy * accelBias[2]);*/
	//float g1 = getG1();
	float x_acceleration = ((cosy * ax + siny * az)) / G_1 * G_1_MPS;
	float y_acceleration = (cosx * ay + -sinx * (-siny * ax + cosy * az)) / G_1 * G_1_MPS;
	float z_acceleration = (sinx * ay + cosx * (-siny * ax + cosy * az)) / G_1 * G_1_MPS;
	
	if(counter % 100 == 0) {
		counter = 0;
		printf("Raw: %.3f %.3f %.3f, Rot: %.3f %.3f %.3f\n", ax ,ay, az, x_acceleration, y_acceleration, z_acceleration);	
	}
	counter++;
	
	z_acceleration -= G_1_MPS;
	
	/*accel.ax -= gx;
	accel.ay -= gy;
	accel.az -= gz;
	accel.ax *= G_1_MPS / G_1;
	accel.ay *= G_1_MPS / G_1;
	accel.az *= G_1_MPS / G_1;*/
	
	return z_acceleration;
}