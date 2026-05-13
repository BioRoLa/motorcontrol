/*
 * abad_calibration.c
 *
 *  Created on: May 8, 2026
 *      Author: Alex
 *
 *  AB/AD (Abduction/Adduction) Hall Effect Calibration Implementation
 */

#include "abad_calibration.h"
#include "hw_config.h"
#include "structs.h"
#include "user_config.h"
#include "math_ops.h"
#include <stdio.h>
#include <string.h>

/* Forward declarations */
static void abad_cal_collect_transitions(FSMStruct * fsmstate);
static void abad_cal_probe_direction(FSMStruct * fsmstate);
static void abad_cal_build_map(FSMStruct * fsmstate);
static void abad_cal_align_zero(FSMStruct * fsmstate);
static int abad_identify_magnet(int sensor_id, int hall_input);

float abad_controller_to_joint_angle(float controller_angle){
    while(controller_angle > PI_F){
        controller_angle -= 2.0f * PI_F;
    }
    while(controller_angle < -PI_F){
        controller_angle += 2.0f * PI_F;
    }
    return controller_angle;
}

float abad_joint_to_controller_angle(float joint_angle){
    while(joint_angle < 0.0f){
        joint_angle += 2.0f * PI_F;
    }
    while(joint_angle >= 2.0f * PI_F){
        joint_angle -= 2.0f * PI_F;
    }
    return joint_angle;
}

float abad_clamp_joint_angle(float joint_angle){
    if(joint_angle < ABAD_LIMIT_MIN_RAD){
        return ABAD_LIMIT_MIN_RAD;
    }
    if(joint_angle > ABAD_LIMIT_MAX_RAD){
        return ABAD_LIMIT_MAX_RAD;
    }
    return joint_angle;
}

uint8_t abad_joint_angle_in_limits(float joint_angle){
    return (joint_angle >= ABAD_LIMIT_MIN_RAD && joint_angle <= ABAD_LIMIT_MAX_RAD);
}

/* Calibration phase state machine */
static uint8_t abad_cal_phase = ABAD_CAL_PHASE_COLLECT_TRANSITIONS;
static int effective_abad_cal_dir = 1;  // Effective calibration direction during collection/alignment

/* Active probe state for startup direction detection */
static uint8_t abad_probe_cycle_count = 0;
static uint8_t abad_probe_attempt_count = 0;
static uint8_t abad_probe_initial_transition_count = 0;
static int abad_probe_dir = 1;

/* Transition history for position map */
typedef struct {
    int sensor_id;        // 0=A, 1=B
    int magnet_id;        // 0=M1, 1=M2, 2=M3, 3=M4
    float recorded_theta; // mechanical angle at transition
} TransitionRecord;

static TransitionRecord transitions[8]; // Up to 4 per sensor
static uint8_t transition_idx = 0;

// Helper macro to convert degrees to radians
#define DEG_TO_RAD(deg) ((deg) * PI_F / 180.0f)

uint8_t abad_sensor_a_active(void) {
    return HAL_GPIO_ReadPin(HALL_A_IO) == 0;  // Active low (0 = magnet detected)
}

uint8_t abad_sensor_b_active(void) {
    return HAL_GPIO_ReadPin(HALL_B_IO) == 0;  // Active low (0 = magnet detected)
}

/*
 * abad_hall_calibrate()
 * Main calibration state machine - runs every interrupt cycle
 */
void abad_hall_calibrate(FSMStruct * fsmstate) {
    if (abad_cal.abad_cal_state == CODE_ABAD_UNCALIBRATED || 
        abad_cal.abad_cal_state >= CODE_ABAD_CAL_SUCCESS) {
        return;  // Not in calibration state
    }

    // Verify motor is configured for AB/AD operation
    if (MOTOR_POSITION == MOTOR_POS_HIP) {
        printf("Error: Motor configured as HIP, cannot run AB/AD calibration\r\n");
        abad_cal.abad_cal_state = CODE_ABAD_CAL_FAIL;
        fsmstate->next_state = MENU_MODE;
        return;
    }

    // Read both sensors
    abad_cal.hall_a_input = HAL_GPIO_ReadPin(HALL_A_IO);
    abad_cal.hall_b_input = HAL_GPIO_ReadPin(HALL_B_IO);

    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);

    // Phase 0: Probe direction at startup
    if (abad_cal_phase == ABAD_CAL_PHASE_PROBE_DIRECTION) {
        abad_cal_probe_direction(fsmstate);
    }
    // Phase 1: Collect transitions from both sensors
    else if (abad_cal_phase == ABAD_CAL_PHASE_COLLECT_TRANSITIONS) {
        // Check for safe-window violation only after direction has been selected
        if ((effective_abad_cal_dir == 1 && joint_theta >= ABAD_SAFE_MAX_RAD) ||
            (effective_abad_cal_dir == -1 && joint_theta <= ABAD_SAFE_MIN_RAD)) {
            abad_cal.abad_cal_state = CODE_ABAD_CAL_FAIL;
            fsmstate->next_state = MENU_MODE;
            printf("\r\nAB/AD Calibration FAILED - exceeded safe travel range\r\n");
            return;
        }
        abad_cal_collect_transitions(fsmstate);
    }
    // Phase 2: Build position map from collected transitions
    else if (abad_cal_phase == ABAD_CAL_PHASE_MAP_POSITIONS) {
        abad_cal_build_map(fsmstate);
    }
    // Phase 3: Align to zero and complete
    else if (abad_cal_phase == ABAD_CAL_PHASE_ALIGN_ZERO) {
        abad_cal_align_zero(fsmstate);
    }

    // Store preinput for next cycle edge detection
    abad_cal.hall_a_preinput = abad_cal.hall_a_input;
    abad_cal.hall_b_preinput = abad_cal.hall_b_input;
}

/* Reset internal calibration state so AB/AD calibration starts cleanly. */
void abad_cal_reset(void){
    transition_idx = 0;
    memset(transitions, 0, sizeof(transitions));
    abad_cal.transition_count = 0;
    abad_cal.hall_a_input = HAL_GPIO_ReadPin(HALL_A_IO);
    abad_cal.hall_b_input = HAL_GPIO_ReadPin(HALL_B_IO);
    abad_cal.hall_a_preinput = abad_cal.hall_a_input;
    abad_cal.hall_b_preinput = abad_cal.hall_b_input;
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(abad_controller_to_joint_angle(controller.theta_mech));
    abad_cal.current_angle_estimate = abad_controller_to_joint_angle(controller.theta_mech);
    abad_cal_phase = ABAD_CAL_PHASE_PROBE_DIRECTION;
    abad_probe_cycle_count = 0;
    abad_probe_attempt_count = 0;
    abad_probe_initial_transition_count = 0;

    // Initialize probe direction from configured direction + motor orientation
    abad_probe_dir = ABAD_CAL_DIR;
    if (MOTOR_POSITION == MOTOR_POS_ABAD_FR_RL) {
        abad_probe_dir = -ABAD_CAL_DIR;
    }
}

/*
 * abad_cal_collect_transitions()
 * Phase 1: Rotate motor slowly and collect magnet transition positions
 */
/*
 * abad_cal_probe_direction()
 * Phase 0: Active bidirectional probe to auto-select working direction
 * Tries configured direction first; if no transitions detected, tries opposite direction.
 */
static void abad_cal_probe_direction(FSMStruct * fsmstate) {
    // On first cycle of this phase, record the current transition count as baseline
    if (abad_probe_cycle_count == 0) {
        abad_probe_initial_transition_count = abad_cal.transition_count;
        printf("AB/AD Cal: probing direction (dir=%d, attempt=%d)...\r\n", abad_probe_dir, abad_probe_attempt_count + 1);
    }

    // Increment probe cycle counter
    abad_probe_cycle_count++;

    // After ABAD_PROBE_CYCLES in one direction, check if we detected transitions
    if (abad_probe_cycle_count > ABAD_PROBE_CYCLES) {
        uint8_t detected_transitions = (abad_cal.transition_count > abad_probe_initial_transition_count);
        
        if (detected_transitions) {
            // Current direction works - move to transition collection
            printf("AB/AD Cal: direction confirmed (dir=%d), collected %d transitions\r\n",
                abad_probe_dir, (abad_cal.transition_count - abad_probe_initial_transition_count));
            effective_abad_cal_dir = abad_probe_dir;
            abad_cal_phase = ABAD_CAL_PHASE_COLLECT_TRANSITIONS;
            abad_probe_cycle_count = 0;
            return;
        } else if (abad_probe_attempt_count == 0) {
            // No transitions detected - try opposite direction
            abad_probe_dir = -abad_probe_dir;
            abad_probe_initial_transition_count = abad_cal.transition_count;
            abad_probe_cycle_count = 0;
            abad_probe_attempt_count = 1;
            printf("AB/AD Cal: no transitions, trying opposite direction (dir=%d)...\r\n", abad_probe_dir);
        } else {
            // Both directions failed to produce hall transitions
            printf("AB/AD Cal: probe failed - no hall transitions detected in either direction\r\n");
            abad_cal.abad_cal_state = CODE_ABAD_CAL_FAIL;
            fsmstate->next_state = MENU_MODE;
            return;
        }
    }

    // Continue probing motion
    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd = joint_pcmd + abad_probe_dir * ABAD_PROBE_STEP_RAD;
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;
}

static void abad_cal_collect_transitions(FSMStruct * fsmstate) {
    // Detect any transition from either sensor
    if ((abad_cal.hall_a_input != abad_cal.hall_a_preinput) ||
        (abad_cal.hall_b_input != abad_cal.hall_b_preinput)) {
        
        // Determine which sensor transitioned and record it
        if (abad_cal.hall_a_input != abad_cal.hall_a_preinput) {
            if (transition_idx < 8) {
                transitions[transition_idx].sensor_id = 0;  // Sensor A
                transitions[transition_idx].recorded_theta = controller.theta_mech;
                // Determine magnet: transitioning to 0 = magnet entering, to 1 = exiting
                transitions[transition_idx].magnet_id = abad_identify_magnet(0, abad_cal.hall_a_input);
                transition_idx++;
            }
        }

        if (abad_cal.hall_b_input != abad_cal.hall_b_preinput) {
            if (transition_idx < 8) {
                transitions[transition_idx].sensor_id = 1;  // Sensor B
                transitions[transition_idx].recorded_theta = controller.theta_mech;
                transitions[transition_idx].magnet_id = abad_identify_magnet(1, abad_cal.hall_b_input);
                transition_idx++;
            }
        }

        abad_cal.transition_count++;
    }

    // Continue rotation: increment position command at calibration speed
    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd = joint_pcmd + effective_abad_cal_dir * (1.0f / 40000.0f) * ABAD_CAL_SPEED;
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);

    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);

    controller.p_des = abad_cal.abad_cal_pcmd;

    // Transition to Phase 2 when we've collected at least 4 transitions
    // (ideally 2 per sensor, or all 4 magnets crossed)
    if (abad_cal.transition_count >= 4) {
        abad_cal_phase = ABAD_CAL_PHASE_MAP_POSITIONS;
        printf("AB/AD Cal: Collected %d transitions, building position map\r\n", abad_cal.transition_count);
    }
}

/*
 * abad_cal_build_map()
 * Phase 2: From collected transitions, calculate position map
 */
static void abad_cal_build_map(FSMStruct * fsmstate) {
    // Initialize position map with collected data
    // Map indices: 0=-90°, 1=-60°, 2=-30°, 3=0°, 4=+30°, 5=+60°, 6=+90°

    uint8_t found_zero = 0;

    // Find A@M1 (0°) and B@M4 (0°) transitions to establish zero
    for (uint8_t i = 0; i < transition_idx; i++) {
        if (transitions[i].sensor_id == 0 && transitions[i].magnet_id == 0) {
            // Sensor A detecting M1
            found_zero = 1;
            break;
        }
    }

    if (!found_zero) {
        // If A@M1 not found, use B@M4
        for (uint8_t i = 0; i < transition_idx; i++) {
            if (transitions[i].sensor_id == 1 && transitions[i].magnet_id == 3) {
                // Sensor B detecting M4
                found_zero = 1;
                break;
            }
        }
    }

    // Build position map by sorting transitions by magnet ID
    // Expected order: M4(A)=-90°, M3(A)=-60°, M2(A)=-30°, M1(A)=0°,
    //                 M4(B)=0°, M3(B)=+30°, M2(B)=+60°, M1(B)=+90°

    // Initialize map
    memset(abad_cal.position_map, 0, sizeof(abad_cal.position_map));

    // Sort transitions and populate map
    float magnet_angles[] = {-90.0f, -60.0f, -30.0f, 0.0f, 30.0f, 60.0f, 90.0f};
    
    for (uint8_t i = 0; i < ABAD_POSITION_MAP_SIZE; i++) {
        abad_cal.position_map[i] = DEG_TO_RAD(magnet_angles[i]);
    }

    // Adjust map based on detected transitions to improve accuracy
    for (uint8_t i = 0; i < transition_idx; i++) {
        // Map each transition to its expected position
        // Sensor A: M4=-90°, M3=-60°, M2=-30°, M1=0°
        // Sensor B: M4=0°, M3=+30°, M2=+60°, M1=+90°
        if (transitions[i].sensor_id == 0) {
            // Sensor A
            float expected_angle = 0.0f - (transitions[i].magnet_id + 1) * 30.0f;
            abad_cal.position_map[ABAD_POS_M90 + transitions[i].magnet_id] = 
                DEG_TO_RAD(expected_angle);
        } else {
            // Sensor B
            float expected_angle = 0.0f + (3 - transitions[i].magnet_id) * 30.0f;
            abad_cal.position_map[ABAD_POS_0 + (3 - transitions[i].magnet_id)] = 
                DEG_TO_RAD(expected_angle);
        }
    }

    abad_cal_phase = ABAD_CAL_PHASE_ALIGN_ZERO;
    printf("AB/AD Cal: Position map built, aligning to zero\r\n");
}

/*
 * abad_cal_align_zero()
 * Phase 3: Move to zero position (A@M1 + B@M4) and establish zero reference
 */
static void abad_cal_align_zero(FSMStruct * fsmstate) {
    // Move to zero position using position control
    // The zero position is where A@M1 = 0° and B@M4 = 0°

    float target_position = abad_clamp_joint_angle(abad_cal.position_map[ABAD_POS_0]);
    float current_position = abad_controller_to_joint_angle(controller.theta_mech);

    // Check if we're close enough to zero (within ±5°)
    float angle_error = fabsf(current_position - target_position);
    if (angle_error > ABAD_CAL_ALIGN_TOL_RAD) {
        // Continue moving toward zero
        float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
        if (current_position < target_position) {
            joint_pcmd += 1.0f / 40000.0f * ABAD_CAL_SPEED;
        } else {
            joint_pcmd -= 1.0f / 40000.0f * ABAD_CAL_SPEED;
        }

        joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
        abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
        
        controller.p_des = abad_cal.abad_cal_pcmd;
    } else {
        // We've reached zero - complete calibration
        abad_cal.abad_cal_pcmd = 0.0f;
        controller.p_des = 0.0f;
        abad_cal.abad_cal_state = CODE_ABAD_CAL_SUCCESS;
        
        // Set encoder zero reference
        abad_encoder_set_zero();
        
        // Transition to motor mode
        fsmstate->next_state = MOTOR_MODE;
        printf("AB/AD Calibration SUCCESS\r\n");
    }
}

/*
 * abad_identify_magnet()
 * Determine which magnet is being detected based on sensor and transition direction
 * Returns magnet ID: 0=M1, 1=M2, 2=M3, 3=M4
 */
static int abad_identify_magnet(int sensor_id, int hall_input) {
    // This is a simplified version. In practice, you'd need to track
    // the sequence of transitions to determine magnet ID.
    // For now, return based on the transition pattern observed during rotation.
    
    // Assumes rotation direction known from ABAD_CAL_DIR
    // and we can infer magnet sequence from transition order
    
    // Simplified: use transition count to estimate magnet
    int magnet = (transition_idx / 2) % 4;
    return magnet;
}

/*
 * abad_encoder_set_zero()
 * Establish mechanical zero for AB/AD axis
 */
void abad_encoder_set_zero(void) {
    // Set the current mechanical position as zero reference
    // Store this reference for future position readings
    controller.theta_mech = 0.0f;
    
    printf("AB/AD zero position set\r\n");
}

/*
 * abad_get_position()
 * Return current AB/AD position based on hall sensor readings
 */
float abad_get_position(void) {
    int hall_a = abad_sensor_a_active();
    int hall_b = abad_sensor_b_active();

    // Determine which magnet(s) are detected
    // Sensor priority: use A if in negative region, B if positive
    
    float current_angle = 0.0f;

    if (hall_a) {
        // Sensor A active - in negative region (or at zero)
        // Use sensor A readings for position
        // This would need to map sensor state to position_map entry
        current_angle = abad_cal.position_map[ABAD_POS_M30];  // Default: -30°
    } else if (hall_b) {
        // Sensor B active - in positive region (or at zero)
        current_angle = abad_cal.position_map[ABAD_POS_P30];  // Default: +30°
    } else {
        // No magnet detected - return last known position
        current_angle = abad_cal.current_angle_estimate;
    }

    abad_cal.current_angle_estimate = current_angle;
    return current_angle;
}
