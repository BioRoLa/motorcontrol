/*
 * hall_calibration.h
 *
 *  Created on: May 8, 2026
 *      Author: Alex
 *
 *  Hall effect calibration routines for both joint types:
 *   - HIP:   single binary hall sensor (hall_calibrate)
 *   - AB/AD: dual binary hall sensors (A, B) over 4 magnets; zero is the center
 *            of the zone where both are active (abad_hall_calibrate)
 *  See HALL_CALIBRATION.md.
 */

#ifndef INC_HALL_CALIBRATION_H_
#define INC_HALL_CALIBRATION_H_

#include "math_ops.h"
#include "fsm.h"
#include "calibration.h"

/* Calibration phases */
#define ABAD_CAL_PHASE_FIND_FIRST_SENSOR    0  // Move up: probe if no sensor is active, back out if in the zone
#define ABAD_CAL_PHASE_FIND_ZERO            1  // Sweep toward the both-active zone (reverses once if on the wrong side)
#define ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE 2 // Reverse and sample both-active trigger in opposite direction
#define ABAD_CAL_PHASE_CENTER_ZERO          3  // Move to midpoint of forward/reverse trigger samples
#define ABAD_CAL_PHASE_BACKOUT              4  // Both sensors active on entry: back out upward to a clean edge

/* Mechanical limits for AB/AD joint */
#define ABAD_LIMIT_MIN_DEG     -85.0f
#define ABAD_LIMIT_MAX_DEG      85.0f

#define ABAD_LIMIT_MIN_RAD    (ABAD_LIMIT_MIN_DEG * PI_F / 180.0f)
#define ABAD_LIMIT_MAX_RAD    (ABAD_LIMIT_MAX_DEG * PI_F / 180.0f)

/* Motor-angle side of the zone where each sensor's inner magnets lie. The hall board and
 * magnets turn with the motor, so this is the same on every AB/AD leg (front and rear,
 * left and right). Measured on FL/RR legs: sensor B passes inner magnets toward +angle.
 * Which side is leg-up depends on the leg (MOTOR_POSITION): FL/RR up = A side,
 * FR/RL up = B side. See HALL_CALIBRATION.md. */
#define ABAD_A_SIDE_DIR   (-1)
#define ABAD_B_SIDE_DIR   (+1)

/* Simplified AB/AD calibration configuration.
 * Step 1: always move up first (the leg can hit the robot stand going down):
 *   both active  -> back out upward until the zone clears, then sweep down into it.
 *   otherwise    -> sweep up (probing <= 30 deg if no sensor is active yet).
 *   While sweeping toward a side, if that side's sensor turns on and off before the zone
 *   (it only sees inner magnets on its own side), reverse once.
 * Step 2: center on the both-active zone.
 * The zero search is bounded by mechanical travel (it fails only if it reaches a
 * mechanical limit without both sensors becoming active), so starting far from zero
 * and passing several magnets on the way in is not an error. */
#define ABAD_PROBE_TRAVEL_DEG    30.0f
#define ABAD_PROBE_TRAVEL_RAD    (ABAD_PROBE_TRAVEL_DEG * PI_F / 180.0f)

/* Blocked-joint check (hip and AB/AD): if the joint lags the calibration command by more
 * than HALL_CAL_BLOCKED_DEG for HALL_CAL_BLOCKED_CYCLES in a row, it's pushing against the
 * stand, a hard stop or something else, so calibration fails and the motor is turned off.
 * Normal tracking lag on the bench is ~1 deg; the SUCCESS line prints the run's max lag. */
#define HALL_CAL_BLOCKED_DEG     8.0f
#define HALL_CAL_BLOCKED_RAD     (HALL_CAL_BLOCKED_DEG * PI_F / 180.0f)
#define HALL_CAL_BLOCKED_CYCLES  4000U   // 0.1 s at 40 kHz

/*
 * hall_calibrate()
 * HIP calibration routine - run every interrupt cycle during HALL_CALIBRATE state (HIP path)
 * Sweeps a single binary hall sensor to find its center, then zeros the encoder there
 */
void hall_calibrate(FSMStruct * fsmstate);
void hall_cal_reset(void);

/*
 * abad_hall_calibrate()
 * Main calibration routine - run every interrupt cycle during HALL_CALIBRATE state (ABAD path)
 * Finds the both-active zone, centers on it, and zeros the encoder there
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

#endif /* INC_HALL_CALIBRATION_H_ */
