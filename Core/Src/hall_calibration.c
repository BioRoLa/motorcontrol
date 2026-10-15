/*
 * hall_calibration.c
 *
 *  Created on: May 8, 2026
 *      Author: Alex
 *
 *  Hall effect calibration implementation for HIP (single sensor) and
 *  AB/AD (dual sensor) joints.
 */

#include "hall_calibration.h"
#include "fsm.h"
#include "hw_config.h"
#include "structs.h"
#include "user_config.h"
#include "math_ops.h"
#include <stdio.h>

/* Forward declarations */
static void abad_cal_find_first_sensor(FSMStruct * fsmstate);
static void abad_cal_backout(FSMStruct * fsmstate);
static void abad_cal_find_zero(FSMStruct * fsmstate);
static void abad_cal_center_sample_reverse(FSMStruct * fsmstate);
static void abad_cal_center_zero(FSMStruct * fsmstate);
static void abad_cal_fail(FSMStruct * fsmstate, const char *message);
static void abad_enter_find_zero(int dir, float joint_theta);
static const char *abad_side_name(int dir);
static const char *abad_phase_name(uint8_t phase);

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

/* Calibration state */
static uint8_t abad_cal_phase = ABAD_CAL_PHASE_FIND_FIRST_SENSOR;
static int abad_motion_dir = 1;
static float abad_phase_start_joint = 0.0f;
static uint8_t abad_last_logged_phase = 0xFF;
static uint16_t abad_phase_log_div = 0;
static float abad_zero_target = 0.0f;
static uint16_t abad_center_settle_count = 0;
static uint8_t abad_prev_both_active = 0;
static float abad_center_trigger_forward = 0.0f;
static uint8_t abad_center_stage = 0;
static float abad_center_far_edge = 0.0f;
static uint8_t abad_first_cycle = 1;
static uint8_t abad_watch_armed = 0;
static uint8_t abad_watch_off = 0;
static float abad_watch_off_joint = 0.0f;
static uint8_t abad_find_zero_reversed = 0;
static uint32_t abad_center_zero_cycles = 0;

#define ABAD_CENTER_SETTLE_CYCLES   2000    // 50 ms held within tolerance at the target
#define ABAD_CENTER_OVERTRAVEL_DEG  8.0f
#define ABAD_CENTER_OVERTRAVEL_RAD  (ABAD_CENTER_OVERTRAVEL_DEG * PI_F / 180.0f)
#define ABAD_CENTER_ZONE_MAX_DEG    60.0f
#define ABAD_CENTER_ZONE_MAX_RAD    (ABAD_CENTER_ZONE_MAX_DEG * PI_F / 180.0f)
#define ABAD_WATCH_DEBOUNCE_DEG     1.5f
#define ABAD_WATCH_DEBOUNCE_RAD     (ABAD_WATCH_DEBOUNCE_DEG * PI_F / 180.0f)

// CENTER_SAMPLE_REVERSE stages
#define ABAD_CENTER_STAGE_CROSS_ZONE   0   // moving through the zone toward its far edge
#define ABAD_CENTER_STAGE_OVERTRAVEL   1   // past the far edge, overtraveling before reverse
#define ABAD_CENTER_STAGE_REVERSE      2   // reversed, waiting to re-enter the zone
#define ABAD_CENTER_FINAL_TOL_DEG   1.0f
#define ABAD_CENTER_FINAL_TOL_RAD   (ABAD_CENTER_FINAL_TOL_DEG * PI_F / 180.0f)
#define ABAD_CENTER_ZERO_TIMEOUT_CYCLES 400000U  // 10 s: slew up to half of ABAD_CENTER_ZONE_MAX_DEG plus settle

/* The pins are read once per cycle into abad_cal.hall_a/b_input (abad_hall_calibrate,
 * abad_cal_reset); everything in that cycle uses the same reading. Active low. */
static uint8_t abad_sensor_a_active(void) {
    return abad_cal.hall_a_input == 0;
}

static uint8_t abad_sensor_b_active(void) {
    return abad_cal.hall_b_input == 0;
}

/*
 * abad_hall_calibrate()
 * Main calibration state machine - runs every interrupt cycle
 */
void abad_hall_calibrate(FSMStruct * fsmstate) {
    if (hall_cal.hall_cal_state == CODE_HALL_UNCALIBRATED ||
        hall_cal.hall_cal_state >= CODE_HALL_CAL_SUCCESS) {
        return;
    }

    if (MOTOR_POSITION == MOTOR_POS_HIP) {
        abad_cal_fail(fsmstate, "AB/AD calibration failed - motor configured as HIP");
        return;
    }

    abad_cal.hall_a_input = HAL_GPIO_ReadPin(HALL_A_IO);
    abad_cal.hall_b_input = HAL_GPIO_ReadPin(HALL_B_IO);

    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);

    uint8_t log_phase_status = 0;
    uint8_t phase_changed = (abad_last_logged_phase != abad_cal_phase);
    abad_phase_log_div++;
    if (phase_changed || (abad_phase_log_div >= 8000)) {
        log_phase_status = 1;
        abad_phase_log_div = 0;
        abad_last_logged_phase = abad_cal_phase;
    }

    if (log_phase_status) {
                float enc_frame_angle_deg = joint_theta * 180.0f / PI_F;
                float delta_from_start_deg = abad_controller_to_joint_angle(joint_theta - abad_phase_start_joint) * 180.0f / PI_F;
                printf("[ABAD CAL] Phase: %s | DeltaFromStart: %.2f deg | EncFrame: %.2f deg | A_active=%d B_active=%d | InnerTransitions=%u\r\n",
               abad_phase_name(abad_cal_phase),
                             (double)delta_from_start_deg,
                             (double)enc_frame_angle_deg,
               (int)abad_sensor_a_active(),
               (int)abad_sensor_b_active(),
               (unsigned)abad_cal.inner_transition_count);
        if (phase_changed) {
            can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
                                controller.i_q_filt*KT*GR, CODE_HALL_CALIBRATING, HALL_CALIBRATE);
        }
    }

    if (abad_cal_phase == ABAD_CAL_PHASE_FIND_FIRST_SENSOR) {
        abad_cal_find_first_sensor(fsmstate);
    } else if (abad_cal_phase == ABAD_CAL_PHASE_BACKOUT) {
        abad_cal_backout(fsmstate);
    } else if (abad_cal_phase == ABAD_CAL_PHASE_FIND_ZERO) {
        abad_cal_find_zero(fsmstate);
    } else if (abad_cal_phase == ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE) {
        abad_cal_center_sample_reverse(fsmstate);
    } else if (abad_cal_phase == ABAD_CAL_PHASE_CENTER_ZERO) {
        abad_cal_center_zero(fsmstate);
    }

    abad_cal.hall_a_preinput = abad_cal.hall_a_input;
    abad_cal.hall_b_preinput = abad_cal.hall_b_input;
}

void abad_cal_reset(void){
    // Force a fresh encoder sample so calibration starts from the live position,
    // even if the motor was manually moved while not commutating.
    ps_sample(&comm_encoder, DT);
    controller.theta_mech = comm_encoder.angle_multiturn[0] / GR;

    abad_cal.hall_a_input = HAL_GPIO_ReadPin(HALL_A_IO);
    abad_cal.hall_b_input = HAL_GPIO_ReadPin(HALL_B_IO);
    abad_cal.hall_a_preinput = abad_cal.hall_a_input;
    abad_cal.hall_b_preinput = abad_cal.hall_b_input;
    float start_joint = abad_controller_to_joint_angle(controller.theta_mech);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(start_joint);
    abad_cal.inner_transition_count = 0;

    abad_cal_phase = ABAD_CAL_PHASE_FIND_FIRST_SENSOR;
    abad_motion_dir = ABAD_A_SIDE_DIR;
    abad_phase_start_joint = start_joint;
    controller.p_des = abad_cal.abad_cal_pcmd;
    abad_last_logged_phase = 0xFF;
    abad_phase_log_div = 0;
    abad_zero_target = 0.0f;
    abad_center_settle_count = 0;
    abad_prev_both_active = (abad_sensor_a_active() && abad_sensor_b_active()) ? 1 : 0;
    abad_center_trigger_forward = 0.0f;
    abad_center_stage = ABAD_CENTER_STAGE_CROSS_ZONE;
    abad_center_far_edge = 0.0f;
    abad_first_cycle = 1;
    abad_watch_armed = 0;
    abad_watch_off = 0;
    abad_watch_off_joint = 0.0f;
    abad_find_zero_reversed = 0;
    abad_center_zero_cycles = 0;
    // FIND_FIRST_SENSOR inspects the live sensor state on its first cycle and picks the
    // correct direction (or backs out of the both-active zone), so no start-position
    // special-casing is needed here.
}

/*
 * Pick the first sweep direction. Everything here is in motor angle, so the same logic
 * works on every AB/AD leg (normal and mirrored): the hall board and magnets turn with
 * the motor. Layout (see HALL_CALIBRATION.md): four magnets, the sensors span the
 * two outer ones, so the zone (both active) is unique. Off the zone, sensor A passes the
 * inner magnets on the A side (ABAD_A_SIDE_DIR) and sensor B passes the ones on the
 * B side. Right at the zone the outer magnet windows don't line up exactly, leaving a
 * sliver of A-only on the B-side edge and B-only on the A-side edge. So one active sensor
 * at start is ambiguous; assume the sliver (zone is adjacent) and let FIND_ZERO's
 * wrong-side rule reverse once if that was wrong:
 *   both active      -> BACKOUT toward the B side, then sweep back toward the A side.
 *   A only at start  -> toward A side (sliver: zone next; inner magnet: reversed later)
 *   B only at start  -> toward B side (same, mirrored)
 *   nothing at start -> probe toward the A side; keep going that way once a sensor appears.
 */
static void abad_cal_find_first_sensor(FSMStruct * fsmstate) {
    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);
    float traveled_mech = fabsf(abad_controller_to_joint_angle(joint_theta - abad_phase_start_joint));
    uint8_t hall_a_active = abad_sensor_a_active();
    uint8_t hall_b_active = abad_sensor_b_active();
    uint8_t both_active = (hall_a_active && hall_b_active);
    uint8_t first_cycle = abad_first_cycle;
    abad_first_cycle = 0;

    if (both_active) {
        abad_motion_dir = ABAD_B_SIDE_DIR;
        abad_cal_phase = ABAD_CAL_PHASE_BACKOUT;
        abad_phase_start_joint = joint_theta;
        printf("AB/AD Cal: both sensors active, backing out toward B side before centering\r\n");
        return;
    }

    if (hall_a_active || hall_b_active) {
        const char *name = hall_a_active ? "A" : "B";
        if (!first_cycle) {
            // Found while probing from an all-off start: keep going the same way.
            printf("AB/AD Cal: %s found while probing, continuing toward A side\r\n", name);
            abad_enter_find_zero(ABAD_A_SIDE_DIR, joint_theta);
        } else {
            int dir = hall_a_active ? ABAD_A_SIDE_DIR : ABAD_B_SIDE_DIR;
            printf("AB/AD Cal: started on %s only, sweeping toward %s\r\n", name, abad_side_name(dir));
            abad_enter_find_zero(dir, joint_theta);
        }
        return;
    }

    if (traveled_mech >= ABAD_PROBE_TRAVEL_RAD) {
        abad_cal_fail(fsmstate, "AB/AD calibration failed - no hall sensor detected within 30 deg");
        return;
    }

    float step = ABAD_CAL_SPEED * DT;
    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd += ABAD_A_SIDE_DIR * step;
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;
}

static void abad_enter_find_zero(int dir, float joint_theta) {
    abad_motion_dir = dir;
    abad_cal_phase = ABAD_CAL_PHASE_FIND_ZERO;
    abad_cal.inner_transition_count = 0;
    abad_phase_start_joint = joint_theta;
    abad_prev_both_active = 0;
    abad_watch_armed = 0;
    abad_watch_off = 0;
    abad_find_zero_reversed = 0;
}

/*
 * Both sensors were active on entry. Move toward the B side until the both-active zone
 * is cleared, then sweep back toward the A side so FIND_ZERO records a clean zone entry
 * as its forward trigger.
 */
static void abad_cal_backout(FSMStruct * fsmstate) {
    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);
    float traveled_mech = fabsf(abad_controller_to_joint_angle(joint_theta - abad_phase_start_joint));
    uint8_t both_active = abad_sensor_a_active() && abad_sensor_b_active();

    if (!both_active) {
        abad_enter_find_zero(ABAD_A_SIDE_DIR, joint_theta);
        printf("AB/AD Cal: cleared both-active zone, sweeping toward A side for center\r\n");
        return;
    }

    if (traveled_mech >= ABAD_PROBE_TRAVEL_RAD) {
        abad_cal_fail(fsmstate, "AB/AD calibration failed - could not exit both-active zone within 30 deg");
        return;
    }

    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd += abad_motion_dir * DT * ABAD_CAL_SPEED;
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;
}

static void abad_cal_find_zero(FSMStruct * fsmstate) {
    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);
    uint8_t hall_a_active = abad_sensor_a_active();
    uint8_t hall_b_active = abad_sensor_b_active();
    uint8_t both_active = (hall_a_active && hall_b_active);
    uint8_t toward_a = (abad_motion_dir == ABAD_A_SIDE_DIR);

    // Diagnostics only: edges on the sensor whose inner magnets lie behind us, i.e. the
    // inner magnets passed on the way in. Starting far out legitimately passes several, so
    // the sweep is bounded by mechanical travel (below) rather than by a count.
    uint8_t inner_transition = toward_a
        ? (abad_cal.hall_b_input != abad_cal.hall_b_preinput)
        : (abad_cal.hall_a_input != abad_cal.hall_a_preinput);
    if (inner_transition && !both_active) {
        abad_cal.inner_transition_count++;
        printf("AB/AD Cal: %s inner-magnet transition %u\r\n", toward_a ? "B" : "A",
               (unsigned)abad_cal.inner_transition_count);
    }

    if (!abad_prev_both_active && both_active) {
        abad_center_trigger_forward = joint_theta;
        abad_prev_both_active = both_active;
        abad_center_stage = ABAD_CENTER_STAGE_CROSS_ZONE;
        abad_cal_phase = ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE;
        printf("AB/AD Cal: forward both-active trigger at %.2f deg, crossing zone to far edge\r\n",
               (double)(abad_center_trigger_forward * 180.0f / PI_F));
        return;
    }
    abad_prev_both_active = both_active;

    // Wrong-side rule. Sensor A only sees inner magnets on the A side of the zone, B only
    // on the B side. Moving toward a side, if that side's sensor turns on and then off
    // again (debounced by travel) before the zone, we are on that side heading away from
    // the zone: reverse once.
    uint8_t watch_active = toward_a ? hall_a_active : hall_b_active;
    if (watch_active) {
        abad_watch_armed = 1;
        abad_watch_off = 0;
    } else if (abad_watch_armed) {
        if (!abad_watch_off) {
            abad_watch_off = 1;
            abad_watch_off_joint = joint_theta;
        } else if (fabsf(abad_controller_to_joint_angle(joint_theta - abad_watch_off_joint)) >= ABAD_WATCH_DEBOUNCE_RAD) {
            if (abad_find_zero_reversed) {
                abad_cal_fail(fsmstate, "AB/AD calibration failed - sensor pattern inconsistent (second wrong-side reversal)");
                return;
            }
            abad_motion_dir = -abad_motion_dir;
            abad_phase_start_joint = joint_theta;
            abad_watch_armed = 0;
            abad_watch_off = 0;
            abad_find_zero_reversed = 1;
            printf("AB/AD Cal: %s dropped before zone, reversing toward %s\r\n",
                   toward_a ? "A" : "B", abad_side_name(abad_motion_dir));
        }
    }

    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd += abad_motion_dir * DT * ABAD_CAL_SPEED;
    if (joint_pcmd > ABAD_LIMIT_MAX_RAD || joint_pcmd < ABAD_LIMIT_MIN_RAD) {
        // Swept the full mechanical travel without ever seeing both sensors active.
        // This is the genuine failure (wrong direction or dead sensor), independent of
        // how many inner magnets were passed on the way.
        abad_cal_fail(fsmstate, "AB/AD calibration failed - reached mechanical limit before both sensors active");
        return;
    }
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;
}

/*
 * Second centering sample. The zone can be wider than any fixed overtravel, so:
 *   1. CROSS_ZONE: keep moving forward until both-active drops (far edge of the zone).
 *   2. OVERTRAVEL: continue past the far edge by ABAD_CENTER_OVERTRAVEL_DEG so the
 *      reverse pass approaches from outside the zone (clears sensor hysteresis).
 *   3. REVERSE:    reverse and capture the both-active rising edge from that side.
 * Both triggers are zone entries, so sensor lag biases them symmetrically and the
 * midpoint is unbiased.
 */
static void abad_cal_center_sample_reverse(FSMStruct * fsmstate) {
    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);
    uint8_t both_active = abad_sensor_a_active() && abad_sensor_b_active();

    if (abad_center_stage == ABAD_CENTER_STAGE_CROSS_ZONE) {
        if (!both_active) {
            abad_center_far_edge = joint_theta;
            abad_center_stage = ABAD_CENTER_STAGE_OVERTRAVEL;
            printf("AB/AD Cal: far zone edge at %.2f deg (zone ~%.2f deg wide), overtraveling %.2f deg\r\n",
                   (double)(abad_center_far_edge * 180.0f / PI_F),
                   (double)(fabsf(abad_controller_to_joint_angle(joint_theta - abad_center_trigger_forward)) * 180.0f / PI_F),
                   (double)ABAD_CENTER_OVERTRAVEL_DEG);
        } else if (fabsf(abad_controller_to_joint_angle(joint_theta - abad_center_trigger_forward)) > ABAD_CENTER_ZONE_MAX_RAD) {
            abad_cal_fail(fsmstate, "AB/AD calibration failed - far zone edge not found within expected width");
            return;
        }
    } else if (abad_center_stage == ABAD_CENTER_STAGE_OVERTRAVEL) {
        if (both_active) {
            // Edge chatter: still at the far edge, keep crossing.
            abad_center_stage = ABAD_CENTER_STAGE_CROSS_ZONE;
        } else if (fabsf(abad_controller_to_joint_angle(joint_theta - abad_center_far_edge)) >= ABAD_CENTER_OVERTRAVEL_RAD) {
            abad_motion_dir = -abad_motion_dir;
            abad_center_stage = ABAD_CENTER_STAGE_REVERSE;
            printf("AB/AD Cal: overtravel complete, reversing for second trigger\r\n");
        }
    } else {
        if (!abad_prev_both_active && both_active) {
            float diff = abad_controller_to_joint_angle(joint_theta - abad_center_trigger_forward);
            abad_zero_target = abad_controller_to_joint_angle(abad_center_trigger_forward + 0.5f * diff);
            abad_cal_phase = ABAD_CAL_PHASE_CENTER_ZERO;
            abad_center_settle_count = 0;
            abad_center_zero_cycles = 0;
            printf("AB/AD Cal: reverse both-active trigger at %.2f deg, center target %.2f deg\r\n",
                   (double)(joint_theta * 180.0f / PI_F),
                   (double)(abad_zero_target * 180.0f / PI_F));
            return;
        }
        // The rising edge should come back within the overtravel distance; allow generous slack.
        if (fabsf(abad_controller_to_joint_angle(joint_theta - abad_center_far_edge)) > ABAD_PROBE_TRAVEL_RAD) {
            abad_cal_fail(fsmstate, "AB/AD calibration failed - reverse trigger not found within expected travel");
            return;
        }
    }
    abad_prev_both_active = both_active;

    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);
    joint_pcmd += abad_motion_dir * DT * ABAD_CAL_SPEED;
    if (joint_pcmd > ABAD_LIMIT_MAX_RAD || joint_pcmd < ABAD_LIMIT_MIN_RAD) {
        abad_cal_fail(fsmstate, "AB/AD calibration failed - reached mechanical limit while centering");
        return;
    }
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;
}

static void abad_cal_center_zero(FSMStruct * fsmstate) {
    float joint_theta = abad_controller_to_joint_angle(controller.theta_mech);
    float error = abad_controller_to_joint_angle(abad_zero_target - joint_theta);
    uint8_t both_active = abad_sensor_a_active() && abad_sensor_b_active();
    float step = ABAD_CAL_SPEED * DT;
    float joint_pcmd = abad_controller_to_joint_angle(abad_cal.abad_cal_pcmd);

    // Slew the command to the computed midpoint at the configured calibration speed, then
    // hold it there. Settling only counts once the command has arrived, otherwise cal
    // completes as soon as the leg is within tolerance and zeroes up to 1 deg short of
    // the target, on the side it approached from.
    float cmd_error = abad_controller_to_joint_angle(abad_zero_target - joint_pcmd);
    uint8_t cmd_at_target = (fabsf(cmd_error) <= step);
    if (cmd_at_target) {
        joint_pcmd = abad_zero_target;
    } else {
        joint_pcmd += (cmd_error > 0.0f) ? step : -step;
    }
    joint_pcmd = abad_clamp_joint_angle(joint_pcmd);
    abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_pcmd);
    controller.p_des = abad_cal.abad_cal_pcmd;

    abad_center_zero_cycles++;
    if (abad_center_zero_cycles > ABAD_CENTER_ZERO_TIMEOUT_CYCLES) {
        abad_cal_fail(fsmstate, "AB/AD calibration failed - center approach timeout");
        return;
    }

    if (cmd_at_target && fabsf(error) <= ABAD_CENTER_FINAL_TOL_RAD && both_active) {
        abad_center_settle_count++;
    } else {
        abad_center_settle_count = 0;
    }

    if (abad_center_settle_count >= ABAD_CENTER_SETTLE_CYCLES) {
        abad_cal.abad_cal_pcmd = abad_joint_to_controller_angle(joint_theta);
        controller.p_des = abad_cal.abad_cal_pcmd;
        hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS;
        encoder_set_zero();
        printf("AB/AD zero position set\r\n");
        fsmstate->next_state = MOTOR_MODE;
        printf("AB/AD Calibration SUCCESS - centered at %.2f deg (target %.2f deg, err %.2f deg)\r\n",
            (double)(joint_theta * 180.0f / PI_F),
            (double)(abad_zero_target * 180.0f / PI_F),
            (double)(error * 180.0f / PI_F));
        can_send_cal_status(0.0f, comm_encoder.velocity/GR, 0.0f,
                            CODE_HALL_CAL_SUCCESS, MOTOR_MODE);
        return;
    }
}

static void abad_cal_fail(FSMStruct * fsmstate, const char *message) {
    hall_cal.hall_cal_state = CODE_HALL_CAL_FAIL;
    controller.p_des = abad_joint_to_controller_angle(abad_controller_to_joint_angle(controller.theta_mech));
    fsmstate->next_state = MENU_MODE;
    printf("%s\r\n", message);
    can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
                        controller.i_q_filt*KT*GR, CODE_HALL_CAL_FAIL, MENU_MODE);
}

static const char *abad_side_name(int dir) {
    return (dir == ABAD_A_SIDE_DIR) ? "A side" : "B side";
}

static const char *abad_phase_name(uint8_t phase) {
    switch (phase) {
        case ABAD_CAL_PHASE_FIND_FIRST_SENSOR:
            return "FIND_FIRST";
        case ABAD_CAL_PHASE_BACKOUT:
            return "BACKOUT";
        case ABAD_CAL_PHASE_FIND_ZERO:
            return "FIND_ZERO";
        case ABAD_CAL_PHASE_CENTER_SAMPLE_REVERSE:
            return "CENTER_REV";
        case ABAD_CAL_PHASE_CENTER_ZERO:
            return "CENTER_ZERO";
        default:
            return "UNKNOWN";
    }
}

 void hall_calibrate(FSMStruct * fsmstate){
     static uint32_t hip_log_div = 0;
     static uint8_t  hip_centering_logged = 0;
     static int      hip_prev_cal_state = -1;

     int state_before = hall_cal.hall_cal_state;

     /* Detect start of a new calibration run */
     if (hip_prev_cal_state != CODE_HALL_CALIBRATING && state_before == CODE_HALL_CALIBRATING) {
         hip_log_div = 0;
         hip_centering_logged = 0;
         printf("HIP Hall Cal: starting  dir=%s  speed=%.1f rad/s\r\n",
                HALL_CAL_DIR > 0 ? "CW" : "CCW", (double)HALL_CAL_SPEED);
         can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
                             controller.i_q_filt*KT*GR, CODE_HALL_CALIBRATING, fsmstate->state);
     }

     if(state_before == CODE_HALL_UNCALIBRATED || state_before >= CODE_HALL_CAL_SUCCESS);
     else{
    	 hall_cal.hall_input = HAL_GPIO_ReadPin(HALL_IO);
    	 if((HALL_CAL_DIR == 1 && controller.theta_mech >= hall_cal.hall_present_pos + 2*PI_F) || (HALL_CAL_DIR == -1 && controller.theta_mech <= hall_cal.hall_present_pos - 2*PI_F)){
    		 hall_cal.hall_cal_state = CODE_HALL_CAL_FAIL;
    		 fsmstate->next_state = MENU_MODE;
    	 }
         else{
        	 if(hall_cal.hall_input != hall_cal.hall_preinput) {
        		 hall_cal.hall_cal_count += 1;
        		 if(hall_cal.hall_input == 0) {
        		     hall_cal.hall_in_pos = controller.theta_mech;
        		     printf("HIP Hall Cal: sensor enter  pos=%.1f deg\r\n",
        		            (double)(controller.theta_mech * 180.0f / PI_F));
        		 } else {
        			 hall_cal.hall_out_pos = controller.theta_mech;
        			 hall_cal.hall_mid_pos = (hall_cal.hall_in_pos + hall_cal.hall_out_pos) / 2.0f;
        			 printf("HIP Hall Cal: sensor exit  pos=%.1f deg  center=%.1f deg\r\n",
        			        (double)(controller.theta_mech * 180.0f / PI_F),
        			        (double)(hall_cal.hall_mid_pos * 180.0f / PI_F));
        			 can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
        			                     controller.i_q_filt*KT*GR, CODE_HALL_CALIBRATING, fsmstate->state);
                 }
             }
             if(hall_cal.hall_cal_count <= 1) {
                 hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*(1.0f/(40000.0f)*HALL_CAL_SPEED);
                 if(++hip_log_div >= 20000) {
                     hip_log_div = 0;
                     printf("HIP Hall Cal: sweeping  pos=%.1f deg\r\n",
                            (double)(controller.theta_mech * 180.0f / PI_F));
                 }
             } else {
                 if(!hip_centering_logged) {
                     hip_centering_logged = 1;
                     printf("HIP Hall Cal: centering to %.1f deg\r\n",
                            (double)(hall_cal.hall_mid_pos * 180.0f / PI_F));
                     can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
                                         controller.i_q_filt*KT*GR, CODE_HALL_CALIBRATING, fsmstate->state);
                 }
                 if(HALL_CAL_DIR == 1){
                     if(HALL_CAL_OFFSET == 0){
                    	 if(controller.theta_mech >= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED;
                    	 else{
                    		 hall_cal.hall_cal_pcmd = 0.0f;
                    		 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS;
                    		 hall_cal.hall_cal_count = 0;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE;
                         }
                     } else {
                         if(controller.theta_mech <= hall_cal.hall_mid_pos + HALL_CAL_OFFSET*PI_F/180) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED;
                         else{
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS;
                             hall_cal.hall_cal_count = 0;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE;
                         }
                     }
                 } else if(HALL_CAL_DIR == -1){
                     if(HALL_CAL_OFFSET == 0){
                         if(controller.theta_mech <= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED;
                         else{
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS;
                             hall_cal.hall_cal_count = 0;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE;
                         }
                     } else {
                         if(controller.theta_mech >= hall_cal.hall_mid_pos - HALL_CAL_OFFSET*PI_F/180) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED;
                         else{
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS;
                             hall_cal.hall_cal_count = 0;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE;
                         }
                     }
                 }
             }
			 if(hall_cal.hall_cal_pcmd > 2*PI_F){
				 hall_cal.hall_cal_pcmd -= 2*PI_F;
			 }
			 if(hall_cal.hall_cal_pcmd < 0){
				 hall_cal.hall_cal_pcmd += 2*PI_F;
			 }
             controller.p_des = hall_cal.hall_cal_pcmd;
         }
         hall_cal.hall_preinput = hall_cal.hall_input;
     }

     /* Report terminal state transitions */
     int state_after = hall_cal.hall_cal_state;
     if (state_before == CODE_HALL_CALIBRATING && state_after != CODE_HALL_CALIBRATING) {
         if (state_after == CODE_HALL_CAL_SUCCESS) {
             printf("HIP Hall Cal: SUCCESS  center=%.1f deg (now zero)\r\n",
                    (double)(hall_cal.hall_mid_pos * 180.0f / PI_F));
         } else {
             printf("HIP Hall Cal: FAIL  swept >360 deg without finding sensor\r\n");
         }
         can_send_cal_status(comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR,
                             controller.i_q_filt*KT*GR, state_after, fsmstate->next_state);
     }
     hip_prev_cal_state = state_after;
 }

