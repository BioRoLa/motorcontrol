/*
 * hall_calibration.h
 *
 *  Created on: May 8, 2026
 *      Author: Alex
 *
 *  Hall effect calibration routines for both joint types:
 *   - HIP:   single binary hall sensor (hall_calibrate)
 *   - AB/AD: dual binary hall sensors (A, B) reading 4 magnets (M1-M4)
 *            on rotor, position range -90 deg (A@M4) to +90 deg (B@M1)
 *            (abad_hall_calibrate)
 */

#ifndef INC_HALL_CALIBRATION_H_
#define INC_HALL_CALIBRATION_H_

#include "math_ops.h"
#include "fsm.h"
#include "calibration.h"

/* Calibration phases */
#define ABAD_CAL_PHASE_FIND_BOTTOM_SENSOR   0  // Move upward until the bottom sensor sees a magnet
#define ABAD_CAL_PHASE_FIND_ZERO            1  // Continue upward until both sensors are active
#define ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE 2 // Reverse and sample both-active trigger in opposite direction
#define ABAD_CAL_PHASE_CENTER_ZERO          3  // Move to midpoint of forward/reverse trigger samples

/* Mechanical limits for AB/AD joint */
#define ABAD_LIMIT_MIN_DEG     -85.0f
#define ABAD_LIMIT_MAX_DEG      85.0f
#define ABAD_SAFE_MIN_DEG      -80.0f
#define ABAD_SAFE_MAX_DEG       80.0f
#define ABAD_CAL_ALIGN_TOL_DEG   5.0f

#define ABAD_LIMIT_MIN_RAD    (ABAD_LIMIT_MIN_DEG * PI_F / 180.0f)
#define ABAD_LIMIT_MAX_RAD    (ABAD_LIMIT_MAX_DEG * PI_F / 180.0f)
#define ABAD_SAFE_MIN_RAD     (ABAD_SAFE_MIN_DEG * PI_F / 180.0f)
#define ABAD_SAFE_MAX_RAD     (ABAD_SAFE_MAX_DEG * PI_F / 180.0f)
#define ABAD_CAL_ALIGN_TOL_RAD (ABAD_CAL_ALIGN_TOL_DEG * PI_F / 180.0f)

/* Simplified AB/AD calibration configuration.
 * Step 1: find the expected bottom sensor within 30 degrees of upward travel.
 * Step 2: keep moving upward until both sensors detect simultaneously.
 * Fail if the bottom sensor toggles more than three times before reaching zero. */
#define ABAD_PROBE_TRAVEL_DEG    30.0f
#define ABAD_PROBE_TRAVEL_RAD    (ABAD_PROBE_TRAVEL_DEG * PI_F / 180.0f)
#define ABAD_MAX_BOTTOM_TRANSITIONS 3

/*
 * hall_calibrate()
 * HIP calibration routine - run every interrupt cycle during HALL_CALIBRATE state (HIP path)
 * Sweeps a single binary hall sensor to find its center, then zeros the encoder there
 */
void hall_calibrate(FSMStruct * fsmstate);

/*
 * abad_hall_calibrate()
 * Main calibration routine - run every interrupt cycle during HALL_CALIBRATE state (ABAD path)
 * Detects magnet transitions from both sensors, builds position map, establishes zero
 */
void abad_hall_calibrate(FSMStruct * fsmstate);

void abad_cal_reset(void);

/*
 * Angle conversion helpers for AB/AD mechanical limits.
 * Joint angles are signed in the range [-pi, pi].
 * Controller angles remain wrapped to [0, 2pi).
 */
float abad_controller_to_joint_angle(float controller_angle);
float abad_joint_to_controller_angle(float joint_angle);
float abad_clamp_joint_angle(float joint_angle);
uint8_t abad_joint_angle_in_limits(float joint_angle);

/*
 * abad_sensor_a_active()
 * Check if sensor A is currently detecting a magnet
 */
uint8_t abad_sensor_a_active(void);

/*
 * abad_sensor_b_active()
 * Check if sensor B is currently detecting a magnet
 */
uint8_t abad_sensor_b_active(void);

#endif /* INC_HALL_CALIBRATION_H_ */
