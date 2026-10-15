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
#define ABAD_CAL_PHASE_FIND_FIRST_SENSOR    0  // Pick a sweep direction from the sensors active at start (probe if none)
#define ABAD_CAL_PHASE_FIND_ZERO            1  // Sweep toward the both-active zone (reverses once if on the wrong side)
#define ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE 2 // Reverse and sample both-active trigger in opposite direction
#define ABAD_CAL_PHASE_CENTER_ZERO          3  // Move to midpoint of forward/reverse trigger samples
#define ABAD_CAL_PHASE_BACKOUT              4  // Both sensors active on entry: back out toward the B side to a clean edge

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

/* Motor-angle side of the zone where each sensor's inner magnets lie. The hall board and
 * magnets turn with the motor, so this is the same on every AB/AD leg; MOTOR_POSITION
 * 1 (FL/RR) vs 2 (FR/RL) does not affect calibration. Measured on FL/RR legs: sensor B
 * passes inner magnets toward +angle. See docs/abad_hall_cal_setup.md. */
#define ABAD_A_SIDE_DIR   (-1)
#define ABAD_B_SIDE_DIR   (+1)

/* Simplified AB/AD calibration configuration.
 * Step 1: pick a direction from the sensors active at start:
 *   both active  -> back out toward the B side to a clean edge, then sweep toward A.
 *   A only       -> sweep toward the A side.   B only -> sweep toward the B side.
 *   neither      -> probe toward the A side (<= 30 deg) until a sensor appears, keep going.
 *   While sweeping toward a side, if that side's sensor turns on and off before the zone
 *   (it only sees inner magnets on its own side), reverse once.
 * Step 2: center on the both-active zone.
 * The zero search is bounded by mechanical travel (it fails only if it reaches a
 * mechanical limit without both sensors becoming active), so starting far from zero
 * and passing several magnets on the way in is not an error. */
#define ABAD_PROBE_TRAVEL_DEG    30.0f
#define ABAD_PROBE_TRAVEL_RAD    (ABAD_PROBE_TRAVEL_DEG * PI_F / 180.0f)

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
