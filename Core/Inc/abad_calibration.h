/*
 * abad_calibration.h
 *
 *  Created on: May 8, 2026
 *      Author: Alex
 *
 *  AB/AD (Abduction/Adduction) Hall Effect Calibration
 *  Dual binary hall sensors (A, B) reading 4 magnets (M1-M4) on rotor
 *  Position range: -90° (A@M4) to +90° (B@M1)
 */

#ifndef INC_ABAD_CALIBRATION_H_
#define INC_ABAD_CALIBRATION_H_

#include "math_ops.h"
#include "fsm.h"
#include "calibration.h"

/* Magnet positions on rotor (fixed, physical) */
#define MAGNET_M1_POS   -45.0f  // degrees
#define MAGNET_M2_POS   -15.0f
#define MAGNET_M3_POS    15.0f
#define MAGNET_M4_POS    45.0f

/* Sensor positions (fixed, physical) */
#define SENSOR_A_POS    -45.0f  // degrees
#define SENSOR_B_POS     45.0f

/* Position map indices (object angle in degrees, converted to radians) */
#define ABAD_POS_M90     0   // -90°: A@M4
#define ABAD_POS_M60     1   // -60°: A@M3
#define ABAD_POS_M30     2   // -30°: A@M2
#define ABAD_POS_0       3   //   0°: A@M1 + B@M4 (zero position)
#define ABAD_POS_P30     4   //  +30°: B@M3
#define ABAD_POS_P60     5   //  +60°: B@M2
#define ABAD_POS_P90     6   //  +90°: B@M1

/* Calibration phases */
#define ABAD_CAL_PHASE_PROBE_DIRECTION      0  // Active direction probe at startup
#define ABAD_CAL_PHASE_COLLECT_TRANSITIONS  1  // Collecting magnet transitions
#define ABAD_CAL_PHASE_MAP_POSITIONS        2  // Building position map
#define ABAD_CAL_PHASE_ALIGN_ZERO           3  // Aligning to zero position

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

/* Active probe configuration: small bidirectional test at startup to auto-select
 * the direction that shows hall activity, regardless of encoder reference accuracy. */
#define ABAD_PROBE_STEP_DEG     5.0f        // Small step size for probing
#define ABAD_PROBE_CYCLES       3           // Number of cycles to attempt each direction
#define ABAD_PROBE_STEP_RAD     (ABAD_PROBE_STEP_DEG * PI_F / 180.0f)

/*
 * abad_hall_calibrate()
 * Main calibration routine - run every interrupt cycle during ABAD_CALIBRATE state
 * Detects magnet transitions from both sensors, builds position map, establishes zero
 */
void abad_hall_calibrate(FSMStruct * fsmstate);

/*
 * abad_encoder_set_zero()
 * Establishes the mechanical zero reference for AB/AD axis
 * Called when calibration completes successfully
 */
void abad_encoder_set_zero(void);

/*
 * abad_get_position()
 * Returns current estimated AB/AD angle based on hall sensor readings
 * Uses position map and sensor priority logic
 * Returns angle in radians, range [-π/2, π/2]
 */
float abad_get_position(void);
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

#endif /* INC_ABAD_CALIBRATION_H_ */
