/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "drv8323.h"
#include "abad_calibration.h"

 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every commutation interrupt cycle */

	 /* state transition management */
	 if(fsmstate->next_state != fsmstate->state){
		 fsm_exit_state(fsmstate);		// safely exit the old state
		 if(fsmstate->ready){			// if the previous state is ready, enter the new state
			 fsmstate->state = fsmstate->next_state;
			 fsm_enter_state(fsmstate);
		 }
	 }

	 switch(fsmstate->state){
		 case MENU_MODE:
			 break;

		 case ENCODER_CALIBRATE:
			 if(!comm_encoder_cal.done_ordering){
				 order_phases(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else if(!comm_encoder_cal.done_cal){
				 calibrate_encoder(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else{
				 /* Exit calibration mode when done */
				 //for(int i = 0; i<128*PPAIRS; i++){printf("%d\r\n", error_array[i]);}
				 E_ZERO = comm_encoder_cal.ezero;
				 printf("E_ZERO: %d  %f\r\n", E_ZERO, TWO_PI_F*fmodf((comm_encoder.ppairs*(float)(-E_ZERO))/((float)ENC_CPR), 1.0f));
				 memcpy(&comm_encoder.offset_lut, comm_encoder_cal.lut_arr, sizeof(comm_encoder.offset_lut));
				 memcpy(&ENCODER_LUT, comm_encoder_cal.lut_arr, sizeof(comm_encoder_cal.lut_arr));
				 //for(int i = 0; i<128; i++){printf("%d\r\n", ENCODER_LUT[i]);}
				 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
				 preference_writer_flush(&prefs);
				 preference_writer_close(&prefs);
				 preference_writer_load(prefs);
				 update_fsm(fsmstate, 27);
			 }

			 break;

		 case MOTOR_MODE:
			 /* If CAN has timed out, reset all commands */
			 if((CAN_TIMEOUT > 0 ) && (controller.timeout > CAN_TIMEOUT)){
				 zero_commands(&controller);
			 }
			 /* Otherwise, commutate */

			 torque_control(&controller);
			 field_weaken(&controller);
			 commutate(&controller, &comm_encoder);

			 controller.timeout ++;
			 break;

		 case SETUP_MODE:
			 break;

		 case ENCODER_MODE:
			 ps_print(&comm_encoder, 100);
			 break;

		 case HALL_CALIBRATE:
			 /* If CAN has timed out, reset all commands */
			 if((CAN_TIMEOUT > 0 ) && (controller.timeout > CAN_TIMEOUT)){
				 zero_commands(&controller);
			 }
			 /* Otherwise, commutate */

			 /* Calibrate Hall Sensor */
			 hall_calibrate(fsmstate);

			 torque_control(&controller);
			 commutate(&controller, &comm_encoder);

			 controller.timeout ++;
			 break;
	 case ABAD_CALIBRATE:
		 /* If CAN has timed out, reset all commands */
		 if((CAN_TIMEOUT > 0 ) && (controller.timeout > CAN_TIMEOUT)){
			 zero_commands(&controller);
		 }
		 /* Otherwise, commutate */

		 /* Calibrate AB/AD Hall Sensor */
		 abad_hall_calibrate(fsmstate);

		 torque_control(&controller);
		 commutate(&controller, &comm_encoder);

		 controller.timeout ++;
		 break;	 }

 }

 void fsm_enter_state(FSMStruct * fsmstate){
	 /* Called when entering a new state
	  * Do necessary setup   */

		switch(fsmstate->state){
			case MENU_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Main Menu\r\n");
				}
				break;
			case SETUP_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Setup\r\n");
				}
				enter_setup_state();
				break;
			case ENCODER_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Encoder Mode\r\n");
				}
				break;
			case MOTOR_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Motor Mode\r\n");
				}
				controller.kp = MOTOR_MODE_KP;
				controller.ki = MOTOR_MODE_KI;
				controller.kd = MOTOR_MODE_KD;
				enter_motor_mode();
				break;
			case ENCODER_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Encoder Calibration Mode\r\n");
				}
				/* zero out all calibrations before starting */

				comm_encoder_cal.done_cal = 0;
				comm_encoder_cal.done_ordering = 0;
				comm_encoder_cal.started = 0;
				comm_encoder.e_zero = 0;
				memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
				drv_enable_gd(drv);
				GPIO_ENABLE;
				break;
			case HALL_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering Hall Calibration Mode\r\n");
				}
				controller.kp = HALL_CAL_KP;
				controller.ki = HALL_CAL_KI;
				controller.kd = HALL_CAL_KD;
				enter_motor_mode();
				break;
			case ABAD_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nEntering AB/AD Hall Calibration Mode\r\n");
				}
				controller.kp = ABAD_CAL_KP;
				controller.ki = ABAD_CAL_KI;
				controller.kd = ABAD_CAL_KD;
				enter_motor_mode();
				abad_cal_reset();
				abad_cal.abad_cal_state = CODE_ABAD_CALIBRATING;
				abad_cal.abad_present_pos = controller.theta_mech;
				break;

		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
			case MENU_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nLeaving Main Menu\r\n");
				}
				fsmstate->ready = 1;
				break;
			case SETUP_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nLeaving Setup Menu\r\n");
				}
				fsmstate->ready = 1;
				break;
			case ENCODER_MODE:
				if (fsmstate->print_uart_msg){
					printf("\r\nLeaving Encoder Mode\r\n");
				}
				fsmstate->ready = 1;
				break;
			case MOTOR_MODE:
				/* Don't stop commutating if there are high currents or FW happening */
				//if( (fabs(controller.i_q_filt)<1.0f) && (fabs(controller.i_d_filt)<1.0f) ){
				if (fsmstate->print_uart_msg){
					printf("\r\nLeaving Motor Mode\r\n");
				}
				fsmstate->ready = 1;
				drv_disable_gd(drv);
				reset_foc(&controller);
				GPIO_DISABLE;
				LED_LOW;
				//}
				zero_commands(&controller);		// Set commands to zero
				break;
			case ENCODER_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nExiting Encoder Calibration Mode\r\n");
				}
				GPIO_DISABLE;
				drv_disable_gd(drv);
				//free(error_array);
				//free(lut_array);

				fsmstate->ready = 1;
				break;
			case HALL_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nExiting Hall Calibration Mode\r\n");
				}
				GPIO_DISABLE;
				LED_LOW;
				drv_disable_gd(drv);
				fsmstate->ready = 1;
				break;
			case ABAD_CALIBRATE:
				if (fsmstate->print_uart_msg){
					printf("\r\nExiting AB/AD Hall Calibration Mode\r\n");
				}
				GPIO_DISABLE;
				LED_LOW;
				drv_disable_gd(drv);
				fsmstate->ready = 1;
				break;
		}

 }

 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == MENU_CMD){	// escape to exit to rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		if (fsmstate->print_uart_msg){
			enter_menu_state();
		}
		return;
	}
	switch(fsmstate->state){
		case MENU_MODE:
			switch (fsm_input){
				case ENCODER_CAL_CMD:
					fsmstate->next_state = ENCODER_CALIBRATE;
					fsmstate->ready = 0;
					break;
				case MOTOR_CMD:
					fsmstate->next_state = MOTOR_MODE;
					fsmstate->ready = 0;
					break;
				case ENCODER_CMD:
					fsmstate->next_state = ENCODER_MODE;
					fsmstate->ready = 0;
					break;
				case SETUP_CMD:
					fsmstate->next_state = SETUP_MODE;
					fsmstate->ready = 0;
					break;
				case ZERO_CMD:
					encoder_set_zero();
					break;
				case HALL_CAL_CMD:
					fsmstate->next_state = HALL_CALIBRATE;
					fsmstate->ready = 0;
					break;
				case ABAD_CAL_CMD:
					if (MOTOR_POSITION == MOTOR_POS_HIP) {
						printf("Error: Motor configured as HIP, cannot run AB/AD calibration\r\n");
					} else {
						fsmstate->next_state = ABAD_CALIBRATE;
						fsmstate->ready = 0;
					}
					break;
				default:
					break;
				}
			break;
		case SETUP_MODE:
			if(fsm_input == ENTER_CMD){
				process_user_input(fsmstate);
				break;
			}
			if(fsmstate->bytecount == 0){fsmstate->cmd_id = fsm_input;}
			else{
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				//fsmstate->bytecount = fsmstate->bytecount%(sizeof(fsmstate->cmd_buff)/sizeof(fsmstate->cmd_buff[0])); // reset when buffer is full
			}
			fsmstate->bytecount++;
			/* If enter is typed, process user input */

			break;

		case ENCODER_MODE:
			break;
		case MOTOR_MODE:
			break;
	}
	//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
 }


 void enter_menu_state(void){
	    //drv.disable_gd();
	    //reset_foc(&controller);
	    //gpio.enable->write(0);
	    printf("\n\r");
	    printf(" Commands:\n\r");
	    printf(" m - Motor Mode\n\r");
	    printf(" c - Calibrate Encoder\n\r");
	    printf(" h - Hall Calibration\n\r");
	    printf(" a - AB/AD Hall Calibration\n\r");
	    printf(" s - Setup\n\r");
	    printf(" e - Display Encoder\n\r");
	    printf(" z - Set Zero Position\n\r");
	    printf(" esc - Exit to Menu\n\r");

	    //gpio.led->write(0);
 }

 void enter_setup_state(void){
	    printf("\r\n Configuration Options \n\r");
	    printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
	    printf("\r\n Motor:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "g", "Gear Ratio",                                "0",   "-",      GR);
	    printf(" %-4s %-31s %-5s %-6s %.5f\n\r", "t", "Torque Constant (N-m/A)",                   "0",   "-",      KT);
		printf(" %-4s %-31s %-5s %-6s %d\n\r",   "P", "Motor Position (0=Hip, 1=FL/RR, 2=FR/RL)",  "0",   "2",      MOTOR_POSITION);
	    printf("\r\n Control:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "b", "Current Bandwidth (Hz)",                    "100", "2000",   I_BW);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "l", "Current Limit (A)",                         "0.0", "75.0",   I_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "p", "Max Position Setpoint (rad)",               "-",   "-",      P_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "v", "Max Velocity Setpoint (rad)/s",             "-",   "-",      V_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "T", "Max Torque Setpoint (rad)/s",               "-",   "-",      T_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "k", "Max Gain for Position (N-m/rad)",           "0.0", "1000.0", KP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "i", "Max Integral Gain for Position (N-m*s/rad)","0.0", "10.0",   KI_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "d", "Max Gain for Velocity (N-m/rad/s)",         "0.0", "5.0",    KD_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "X", "Motor Mode Default Kp (N-m/rad)",          "0.0", "1000.0", MOTOR_MODE_KP);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "Y", "Motor Mode Default Ki (N-m*s/rad)",        "0.0", "10.0",   MOTOR_MODE_KI);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "Z", "Motor Mode Default Kd (N-m/rad/s)",        "0.0", "5.0",    MOTOR_MODE_KD);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "f", "FW Current Limit (A)",                      "0.0", "33.0",   I_FW_MAX);
//	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)",                "0",   "150",    TEMP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "c", "Continuous Current (A)",                    "0.0", "40.0",   I_MAX_CONT);
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "a", "Calibration Current (A)",                   "0.0", "20.0",   I_CAL);
		printf(" %-4s %-31s %-5s %-6s %d\n\r",   "r", "Hall Calibration Direction",                "-1",  "1",      HALL_CAL_DIR);
		printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "e", "Hall Calibration offset",                   "0.0", "143.0",  HALL_CAL_OFFSET);
		printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Hall Calibration Speed",                    "0.0", "10.0",   HALL_CAL_SPEED);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "K", "Hall Calibration Kp (N-m/rad)",            "0.0", "1000.0", HALL_CAL_KP);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "I", "Hall Calibration Ki (N-m*s/rad)",          "0.0", "10.0",   HALL_CAL_KI);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "D", "Hall Calibration Kd (N-m/rad/s)",          "0.0", "5.0",    HALL_CAL_KD);
		printf(" %-4s %-31s %-5s %-6s %d\n\r",   "R", "AB/AD Calibration Direction",              "-1",  "1",      ABAD_CAL_DIR);
		printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "O", "AB/AD Calibration Offset",                 "0.0", "90.0",   ABAD_CAL_OFFSET);
		printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "S", "AB/AD Calibration Speed",                  "0.0", "10.0",   ABAD_CAL_SPEED);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "J", "AB/AD Calibration Kp (N-m/rad)",          "0.0", "1000.0", ABAD_CAL_KP);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "U", "AB/AD Calibration Ki (N-m*s/rad)",        "0.0", "10.0",   ABAD_CAL_KI);
		printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "V", "AB/AD Calibration Kd (N-m/rad/s)",        "0.0", "5.0",    ABAD_CAL_KD);
	    printf("\r\n CAN:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "n", "CAN ID",                                    "0",   "127",    CAN_ID);
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN TX ID",                                 "0",   "127",    CAN_MASTER);
	    printf(" %-4s %-31s %-5s %-6s %d\n\r",   "o", "CAN Timeout (cycles)(0 = none)",            "0",   "100000", CAN_TIMEOUT);
	    printf(" \n\r To change a value, type 'prefix''value''ENTER'\n\r e.g. 'b1000''ENTER'\r\n ");
	    printf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \n\r\n\r");
 }

 void process_user_input(FSMStruct * fsmstate){
	 /* Collects user input from serial (maybe eventually CAN) and updates settings */

	 static char* response;  // Static buffer for the response
	 response = float_reg_update_uart(fsmstate->cmd_id, fsmstate->cmd_buff);
	 if (strcmp(response, STR_INVALID_CMD) == 0){
		 response = int_reg_update_uart(fsmstate->cmd_id, fsmstate->cmd_buff);
	 }
	 printf(response);

	 /*
	 switch (fsmstate->cmd_id){
		 case 'g':
			 GR = fmaxf(atof(fsmstate->cmd_buff), .001f);	// Limit prevents divide by zero if user tries to enter zero
			 printf("GR set to %f\r\n", GR);
			 break;
		 case 't':
			 KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			 printf("KT set to %f\r\n", KT);
			 break;
		 case 'b':
			 I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 100.0f);
			 printf("I_BW set to %f\r\n", I_BW);
			 break;
		 case 'l':
			 I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 75.0f), 0.0f);
			 printf("I_MAX set to %f\r\n", I_MAX);
			 break;
		 case 'p':
			 P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 P_MIN = 0;
			 printf("P_MAX set to %f\r\n", P_MAX);
			 break;
		 case 'v':
			 V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 V_MIN = -V_MAX;
			 printf("V_MAX set to %f\r\n", V_MAX);
			 break;
		 case 'k':
			 KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KP_MAX set to %f\r\n", KP_MAX);
			 break;
		 case 'i':
			 KI_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KI_MAX set to %f\r\n", KI_MAX);
			 break;
		 case 'd':
			 KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KD_MAX set to %f\r\n", KD_MAX);
			 break;
		 case 'f':
			 I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
			 printf("I_FW_MAX set to %f\r\n", I_FW_MAX);
			 break;
		 case 'c':
			 I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			 printf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
			 break;
		 case 'a':
			 I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
			 printf("I_CAL set to %f\r\n", I_CAL);
			 break;
		 case 'r':
			 HALL_CAL_DIR = fmaxf(fminf(atof(fsmstate->cmd_buff), 1.0f), -1.0f);
			 printf("HALL_CAL_DIR set to %d\r\n", HALL_CAL_DIR);
			 break;
		 case 'e':
			 HALL_CAL_OFFSET = fmaxf(fminf(atof(fsmstate->cmd_buff), 143.0f), 0.0f);
			 printf("HALL_CAL_OFFSET set to %f\r\n", HALL_CAL_OFFSET);
			 break;
		 case 's':
			 HALL_CAL_SPEED = fmaxf(fminf(atof(fsmstate->cmd_buff), 10.0f), 0.0f);
			 printf("HALL_CAL_SPEED set to %f\r\n", HALL_CAL_SPEED);
			 break;
		 case 'n':
			 CAN_ID = atoi(fsmstate->cmd_buff);
			 printf("CAN_ID set to %d\r\n", CAN_ID);
			 break;
		 case 'm':
			 CAN_MASTER = atoi(fsmstate->cmd_buff);
			 printf("CAN_TX_ID set to %d\r\n", CAN_MASTER);
			 break;
		 case 'o':
			 CAN_TIMEOUT = atoi(fsmstate->cmd_buff);
			 printf("CAN_TIMEOUT set to %d\r\n", CAN_TIMEOUT);
			 break;
//		 case 'h':
//			 TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
//			 printf("TEMP_MAX set to %f\r\n", TEMP_MAX);
//			 break;
		 default:
			 printf("\n\r '%s' Not a valid command prefix\n\r\n\r", fsmstate->cmd_buff);
			 break;

		 }
	*/

	 /* Write new settings to flash */

	 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
	 preference_writer_flush(&prefs);
	 preference_writer_close(&prefs);
	 preference_writer_load(prefs);

	 enter_setup_state();

	 fsmstate->bytecount = 0;
	 fsmstate->cmd_id = 0;
	 memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
 }

 void enter_motor_mode(void){
	float _f_round, _f_p_des;
	_f_p_des = controller.theta_mech;
	modff(_f_p_des/(2*PI_F),&_f_round);
	_f_p_des = _f_p_des - _f_round*2*PI_F;
	if(_f_p_des < 0) _f_p_des = _f_p_des + 2*PI_F;
	controller.p_des = _f_p_des;

	GPIO_ENABLE;
	LED_HIGH;
	reset_foc(&controller);
	drv_enable_gd(drv);
 }


 void encoder_set_zero(void){
	comm_encoder.m_zero = 0;
	comm_encoder.first_sample = 0;
	int zero_count = comm_encoder.count;
	M_ZERO = zero_count;
	ps_sample(&comm_encoder, DT);
	controller.theta_mech = 0;
 }


 void hall_calibrate(FSMStruct * fsmstate){
     if(hall_cal.hall_cal_state == CODE_HALL_UNCALIBRATED || hall_cal.hall_cal_state >= CODE_HALL_CAL_SUCCESS );
     else{
    	 // read hall sensor
    	 hall_cal.hall_input = HAL_GPIO_ReadPin(HALL_IO);
    	 // calculate new position
    	 if((HALL_CAL_DIR == 1 && controller.theta_mech >= hall_cal.hall_present_pos + 2*PI_F) || (HALL_CAL_DIR == -1 && controller.theta_mech <= hall_cal.hall_present_pos - 2*PI_F)){
    		 hall_cal.hall_cal_state = CODE_HALL_CAL_FAIL ;
    		 fsmstate->next_state = MENU_MODE ;
    	 }
         else{
        	 // rotate the motor forward and backward to read the hall sensor (1: no magnet detected, 0: magnet detected)
        	 // record the position at the moment from 1 to 0 (in_pos), and keep rotating
        	 // record the position at the moment from 0 to 1 (out_pos), and stop rotating.
        	 // calculate the average value of in_pos and out_pos, and rotate the motor to that position slowly
        	 if(hall_cal.hall_input != hall_cal.hall_preinput ) {
        		 hall_cal.hall_cal_count += 1 ;
        		 if(hall_cal.hall_input == 0) hall_cal.hall_in_pos = controller.theta_mech ;
        		 else{
        			 hall_cal.hall_out_pos = controller.theta_mech ;
        			 hall_cal.hall_mid_pos = (hall_cal.hall_in_pos + hall_cal.hall_out_pos)/2.0f ;
                 }
             }
             if(hall_cal.hall_cal_count <= 1) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*(1.0f/(40000.0f)*HALL_CAL_SPEED ) ;
             else{
                 if(HALL_CAL_DIR == 1 ){
                     if(HALL_CAL_OFFSET == 0){
                    	 // keep turning
                    	 if(controller.theta_mech >= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                    	 else{
                    		 // stop
                    		 hall_cal.hall_cal_pcmd = 0.0f;
                    		 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS; // success
                             // zero
                    		 hall_cal.hall_cal_count = 0 ;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE ;
                         }
                     }
                     else{
                         if(controller.theta_mech <= hall_cal.hall_mid_pos + HALL_CAL_OFFSET*PI_F/180)  hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                         else{
                        	 // stop
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS; // success
                             // zero
                             hall_cal.hall_cal_count = 0 ;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE ;
                         }
                     }
                 }
                 else if(HALL_CAL_DIR == -1){
                     if(HALL_CAL_OFFSET == 0){
                    	 // keep turning
                         if(controller.theta_mech <= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                         else{
                        	 // stop
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS; // success
                             // zero
                             hall_cal.hall_cal_count = 0 ;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE ;
                         }
                     }
                     else{
                    	 // calibrate_offset != 0
                         if(controller.theta_mech >= hall_cal.hall_mid_pos - HALL_CAL_OFFSET*PI_F/180)  hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                         else{
                        	 // stop
                        	 hall_cal.hall_cal_pcmd = 0.0f;
                        	 hall_cal.hall_cal_state = CODE_HALL_CAL_SUCCESS; // success
                             // zero
                             hall_cal.hall_cal_count = 0 ;
                    		 encoder_set_zero();
                    		 fsmstate->next_state = MOTOR_MODE ;
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
             controller.p_des = hall_cal.hall_cal_pcmd ;
         }
         hall_cal.hall_preinput = hall_cal.hall_input ;
     }
 }

