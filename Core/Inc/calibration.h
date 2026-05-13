/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "foc.h"

#define V_CAL 	1.0f							// Calibration voltage
//#define I_CAL	5.0f
#define W_CAL 	10.0f							// Calibration speed in rad/s
#define T1		1.0f							// Cal settling period
#define PPAIRS_MAX 64
#define SAMPLES_PER_PPAIR N_LUT
#define N_CAL SAMPLES_PER_PPAIR*PPAIRS_MAX		// Calibration lookup table maximum length, so I don't have to deal with dynamic allocation based on number of pole pairs

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	int offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float theta_start;								// cal start angle
	int ezero;
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, done_rl;		// flags for different cals finished
	uint16_t sample_count;							// keep track of how many samples taken
	float next_sample_time;							// time to take next sample
	int error_arr[N_CAL];
	int lut_arr[N_LUT];
	EncoderStruct cal_position;						// Position reference used for calibration

} CalStruct;

typedef struct{
	int hall_input;
	int hall_preinput;
	float hall_cal_pcmd;
	float hall_cal_speed; // rad/s
	float hall_present_pos; // calibrate the previous position of encoder
	float hall_in_pos;  // read the position from 1 to 0 (magnet enters the area of hall sensor)
	float hall_out_pos; // read the position from 0 to 1 (magnet exits the area of hall sensor)
	float hall_mid_pos;
	float hall_cal_offset; // rad
	int hall_cal_count;
	int hall_cal_state;
} HallCalStruct;

// AB/AD Hall Calibration Structure
// Dual sensors (A, B) reading 4 magnets (M1-M4) at 30° spacing
// Position range: -90° (A@M4) to +90° (B@M1)
#define ABAD_MAGNET_COUNT 4
#define ABAD_POSITION_MAP_SIZE 7

typedef struct{
	int hall_a_input;
	int hall_a_preinput;
	int hall_b_input;
	int hall_b_preinput;
	float abad_cal_pcmd;
	float abad_cal_speed; // rad/s
	float abad_present_pos; // calibration start position
	uint8_t transition_count; // count of detected magnet transitions
	uint8_t active_sensor; // 0=A, 1=B, 2=either
	float position_map[ABAD_POSITION_MAP_SIZE]; // [-90, -60, -30, 0, 30, 60, 90] degrees in radians
	float current_angle_estimate; // Current estimated angle from hall sensor readings
	int abad_cal_state;
} HallCalStructABAD;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,
		int loop_count);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,  int loop_count);

//extern int *error_array;
//extern int *lut_array;

#endif /* INC_CALIBRATION_H_ */
