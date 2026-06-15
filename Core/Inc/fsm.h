/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


// CAN and UART
#define MENU_MODE           0
#define HALL_CALIBRATE      1
#define MOTOR_MODE          2
#define SET_ZERO            3
// UART only
#define SETUP_MODE          4
#define ENCODER_CALIBRATE   5
#define ENCODER_MODE        6
#define HALL_DEBUG_MODE     8

#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define ENCODER_CAL_CMD		'c'
#define ENCODER_CMD			'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define ENTER_CMD			13
#define HALL_CAL_CMD		'h'
#define ABAD_CAL_CMD		'a'
#define HALL_DEBUG_CMD      'q'
#define MOTOR_POS_CMD       'p'


typedef struct{
	uint8_t state;
	uint8_t next_state;
	uint8_t state_change;
	uint8_t ready;
	uint8_t print_uart_msg;
	char cmd_buff[8];
	char bytecount;
	char cmd_id;
}FSMStruct;

void run_fsm(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_motor_mode(void);
void process_user_input(FSMStruct * fsmstate);
void encoder_set_zero(void);
void hall_calibrate(FSMStruct * fsmstate);
void hall_debug_mode(FSMStruct * fsmstate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
