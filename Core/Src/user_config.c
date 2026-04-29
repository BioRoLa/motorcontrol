/*
 * user_config.c
 *
 *  Created on: Jan 10, 2024
 *      Author: yisyuan
 */

#include "structs.h"
#include "user_config.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

struct FloatRegConfig float_reg_config[FLOAT_REG_LENGTH];
struct IntRegConfig int_reg_config[INT_REG_LENGTH];

void user_config_initialize(void){
	for (int i=0; i<FLOAT_REG_LENGTH; i++){
		float_reg_config[i].name = "";
		float_reg_config[i].cmd = ' ';
		float_reg_config[i].f_MIN = 0;
		float_reg_config[i].f_MAX = 0;
	}

	for (int i=0; i<INT_REG_LENGTH; i++){
		int_reg_config[i].name = "";
		int_reg_config[i].cmd = ' ';
		int_reg_config[i].i_MIN = 0;
		int_reg_config[i].i_MAX = 0;
	}

	// initialize the float_reg_config array
	float_reg_config[ADDR_I_BW].name				= NAME_I_BW;
	float_reg_config[ADDR_I_BW].cmd					= CMD_I_BW;
	float_reg_config[ADDR_I_BW].f_MIN				= MIN_I_BW;
	float_reg_config[ADDR_I_BW].f_MAX				= MAX_I_BW;

	float_reg_config[ADDR_I_MAX].name				= NAME_I_MAX;
	float_reg_config[ADDR_I_MAX].cmd				= CMD_I_MAX;
	float_reg_config[ADDR_I_MAX].f_MIN				= MIN_I_MAX;
	float_reg_config[ADDR_I_MAX].f_MAX				= MAX_I_MAX;

	float_reg_config[ADDR_THETA_MIN].name			= NAME_THETA_MIN;
	float_reg_config[ADDR_THETA_MIN].cmd			= CMD_THETA_MIN;
	float_reg_config[ADDR_THETA_MIN].f_MIN			= MIN_THETA_MIN;
	float_reg_config[ADDR_THETA_MIN].f_MAX			= MAX_THETA_MIN;

	float_reg_config[ADDR_THETA_MAX].name			= NAME_THETA_MAX;
	float_reg_config[ADDR_THETA_MAX].cmd			= CMD_THETA_MAX;
	float_reg_config[ADDR_THETA_MAX].f_MIN			= MIN_THETA_MAX;
	float_reg_config[ADDR_THETA_MAX].f_MAX			= MAX_THETA_MAX;

	float_reg_config[ADDR_I_FW_MAX].name			= NAME_I_FW_MAX;
	float_reg_config[ADDR_I_FW_MAX].cmd				= CMD_I_FW_MAX;
	float_reg_config[ADDR_I_FW_MAX].f_MIN			= MIN_I_FW_MAX;
	float_reg_config[ADDR_I_FW_MAX].f_MAX			= MAX_I_FW_MAX;

	float_reg_config[ADDR_R_NOMINAL].name			= NAME_R_NOMINAL;
	float_reg_config[ADDR_R_NOMINAL].cmd			= CMD_R_NOMINAL;
	float_reg_config[ADDR_R_NOMINAL].f_MIN			= MIN_R_NOMINAL;
	float_reg_config[ADDR_R_NOMINAL].f_MAX			= MAX_R_NOMINAL;

	float_reg_config[ADDR_TEMP_MAX].name			= NAME_TEMP_MAX;
	float_reg_config[ADDR_TEMP_MAX].cmd				= CMD_TEMP_MAX;
	float_reg_config[ADDR_TEMP_MAX].f_MIN			= MIN_TEMP_MAX;
	float_reg_config[ADDR_TEMP_MAX].f_MAX			= MAX_TEMP_MAX;

	float_reg_config[ADDR_I_MAX_CONT].name			= NAME_I_MAX_CONT;
	float_reg_config[ADDR_I_MAX_CONT].cmd			= CMD_I_MAX_CONT;
	float_reg_config[ADDR_I_MAX_CONT].f_MIN			= MIN_I_MAX_CONT;
	float_reg_config[ADDR_I_MAX_CONT].f_MAX			= MAX_I_MAX_CONT;

	float_reg_config[ADDR_PPAIRS].name				= NAME_PPAIRS;
	float_reg_config[ADDR_PPAIRS].cmd				= CMD_PPAIRS;
	float_reg_config[ADDR_PPAIRS].f_MIN				= MIN_PPAIRS;
	float_reg_config[ADDR_PPAIRS].f_MAX				= MAX_PPAIRS;

	//	float_reg_config[ADDR_L_D].name				= NAME_L_D;
	//	float_reg_config[ADDR_L_D].cmd				= CMD_L_D;
	//	float_reg_config[ADDR_L_D].f_MIN			= MIN_L_D;
	//	float_reg_config[ADDR_L_D].f_MAX			= MAX_L_D;

	//	float_reg_config[ADDR_L_Q].name				= NAME_L_Q;
	//	float_reg_config[ADDR_L_Q].cmd				= CMD_L_Q;
	//	float_reg_config[ADDR_L_Q].f_MIN			= MIN_L_Q;
	//	float_reg_config[ADDR_L_Q].f_MAX			= MAX_L_Q;

	float_reg_config[ADDR_R_PHASE].name				= NAME_R_PHASE;
	float_reg_config[ADDR_R_PHASE].cmd				= CMD_R_PHASE;
	float_reg_config[ADDR_R_PHASE].f_MIN			= MIN_R_PHASE;
	float_reg_config[ADDR_R_PHASE].f_MAX			= MAX_R_PHASE;

	float_reg_config[ADDR_KT].name					= NAME_KT;
	float_reg_config[ADDR_KT].cmd					= CMD_KT;
	float_reg_config[ADDR_KT].f_MIN					= MIN_KT;
	float_reg_config[ADDR_KT].f_MAX					= MAX_KT;

	float_reg_config[ADDR_R_TH].name				= NAME_R_TH;
	float_reg_config[ADDR_R_TH].cmd					= CMD_R_TH;
	float_reg_config[ADDR_R_TH].f_MIN				= MIN_R_TH;
	float_reg_config[ADDR_R_TH].f_MAX				= MAX_R_TH;

	float_reg_config[ADDR_C_TH].name				= NAME_C_TH;
	float_reg_config[ADDR_C_TH].cmd					= CMD_C_TH;
	float_reg_config[ADDR_C_TH].f_MIN				= MIN_C_TH;
	float_reg_config[ADDR_C_TH].f_MAX				= MAX_C_TH;

	float_reg_config[ADDR_GR].name					= NAME_GR;
	float_reg_config[ADDR_GR].cmd					= CMD_GR;
	float_reg_config[ADDR_GR].f_MIN					= MIN_GR;
	float_reg_config[ADDR_GR].f_MAX					= MAX_GR;

	float_reg_config[ADDR_P_MIN].name				= NAME_P_MIN;
	float_reg_config[ADDR_P_MIN].cmd				= CMD_P_MIN;
	float_reg_config[ADDR_P_MIN].f_MIN				= MIN_P_MIN;
	float_reg_config[ADDR_P_MIN].f_MAX				= MAX_P_MIN;

	float_reg_config[ADDR_P_MAX].name				= NAME_P_MAX;
	float_reg_config[ADDR_P_MAX].cmd				= CMD_P_MAX;
	float_reg_config[ADDR_P_MAX].f_MIN				= MIN_P_MAX;
	float_reg_config[ADDR_P_MAX].f_MAX				= MAX_P_MAX;

	float_reg_config[ADDR_V_MIN].name				= NAME_V_MIN;
	float_reg_config[ADDR_V_MIN].cmd				= CMD_V_MIN;
	float_reg_config[ADDR_V_MIN].f_MIN				= MIN_V_MIN;
	float_reg_config[ADDR_V_MIN].f_MAX				= MAX_V_MIN;

	float_reg_config[ADDR_V_MAX].name				= NAME_V_MAX;
	float_reg_config[ADDR_V_MAX].cmd				= CMD_V_MAX;
	float_reg_config[ADDR_V_MAX].f_MIN				= MIN_V_MAX;
	float_reg_config[ADDR_V_MAX].f_MAX				= MAX_V_MAX;

	float_reg_config[ADDR_T_MIN].name				= NAME_T_MIN;
	float_reg_config[ADDR_T_MIN].cmd				= CMD_T_MIN;
	float_reg_config[ADDR_T_MIN].f_MIN				= MIN_T_MIN;
	float_reg_config[ADDR_T_MIN].f_MAX				= MAX_T_MIN;

	float_reg_config[ADDR_T_MAX].name				= NAME_T_MAX;
	float_reg_config[ADDR_T_MAX].cmd				= CMD_T_MAX;
	float_reg_config[ADDR_T_MAX].f_MIN				= MIN_T_MAX;
	float_reg_config[ADDR_T_MAX].f_MAX				= MAX_T_MAX;

	float_reg_config[ADDR_KP_MAX].name				= NAME_KP_MAX;
	float_reg_config[ADDR_KP_MAX].cmd				= CMD_KP_MAX;
	float_reg_config[ADDR_KP_MAX].f_MIN				= MIN_KP_MAX;
	float_reg_config[ADDR_KP_MAX].f_MAX				= MAX_KP_MAX;

	float_reg_config[ADDR_KI_MAX].name				= NAME_KI_MAX;
	float_reg_config[ADDR_KI_MAX].cmd				= CMD_KI_MAX;
	float_reg_config[ADDR_KI_MAX].f_MIN				= MIN_KI_MAX;
	float_reg_config[ADDR_KI_MAX].f_MAX				= MAX_KI_MAX;

	float_reg_config[ADDR_KD_MAX].name				= NAME_KD_MAX;
	float_reg_config[ADDR_KD_MAX].cmd				= CMD_KD_MAX;
	float_reg_config[ADDR_KD_MAX].f_MIN				= MIN_KD_MAX;
	float_reg_config[ADDR_KD_MAX].f_MAX				= MAX_KD_MAX;

	float_reg_config[ADDR_HALL_CAL_OFFSET].name		= NAME_HALL_CAL_OFFSET;
	float_reg_config[ADDR_HALL_CAL_OFFSET].cmd		= CMD_HALL_CAL_OFFSET;
	float_reg_config[ADDR_HALL_CAL_OFFSET].f_MIN	= MIN_HALL_CAL_OFFSET;
	float_reg_config[ADDR_HALL_CAL_OFFSET].f_MAX	= MAX_HALL_CAL_OFFSET;

	float_reg_config[ADDR_HALL_CAL_SPEED].name		= NAME_HALL_CAL_SPEED;
	float_reg_config[ADDR_HALL_CAL_SPEED].cmd		= CMD_HALL_CAL_SPEED;
	float_reg_config[ADDR_HALL_CAL_SPEED].f_MIN		= MIN_HALL_CAL_SPEED;
	float_reg_config[ADDR_HALL_CAL_SPEED].f_MAX		= MAX_HALL_CAL_SPEED;

	float_reg_config[ADDR_HALL_CAL_KP].name			= NAME_HALL_CAL_KP;
	float_reg_config[ADDR_HALL_CAL_KP].cmd			= CMD_HALL_CAL_KP;
	float_reg_config[ADDR_HALL_CAL_KP].f_MIN			= MIN_HALL_CAL_KP;
	float_reg_config[ADDR_HALL_CAL_KP].f_MAX			= MAX_HALL_CAL_KP;

	float_reg_config[ADDR_HALL_CAL_KI].name			= NAME_HALL_CAL_KI;
	float_reg_config[ADDR_HALL_CAL_KI].cmd			= CMD_HALL_CAL_KI;
	float_reg_config[ADDR_HALL_CAL_KI].f_MIN			= MIN_HALL_CAL_KI;
	float_reg_config[ADDR_HALL_CAL_KI].f_MAX			= MAX_HALL_CAL_KI;

	float_reg_config[ADDR_HALL_CAL_KD].name			= NAME_HALL_CAL_KD;
	float_reg_config[ADDR_HALL_CAL_KD].cmd			= CMD_HALL_CAL_KD;
	float_reg_config[ADDR_HALL_CAL_KD].f_MIN			= MIN_HALL_CAL_KD;
	float_reg_config[ADDR_HALL_CAL_KD].f_MAX			= MAX_HALL_CAL_KD;

	float_reg_config[ADDR_MOTOR_MODE_KP].name		= NAME_MOTOR_MODE_KP;
	float_reg_config[ADDR_MOTOR_MODE_KP].cmd		= CMD_MOTOR_MODE_KP;
	float_reg_config[ADDR_MOTOR_MODE_KP].f_MIN		= MIN_MOTOR_MODE_KP;
	float_reg_config[ADDR_MOTOR_MODE_KP].f_MAX		= MAX_MOTOR_MODE_KP;

	float_reg_config[ADDR_MOTOR_MODE_KI].name		= NAME_MOTOR_MODE_KI;
	float_reg_config[ADDR_MOTOR_MODE_KI].cmd		= CMD_MOTOR_MODE_KI;
	float_reg_config[ADDR_MOTOR_MODE_KI].f_MIN		= MIN_MOTOR_MODE_KI;
	float_reg_config[ADDR_MOTOR_MODE_KI].f_MAX		= MAX_MOTOR_MODE_KI;

	float_reg_config[ADDR_MOTOR_MODE_KD].name		= NAME_MOTOR_MODE_KD;
	float_reg_config[ADDR_MOTOR_MODE_KD].cmd		= CMD_MOTOR_MODE_KD;
	float_reg_config[ADDR_MOTOR_MODE_KD].f_MIN		= MIN_MOTOR_MODE_KD;
	float_reg_config[ADDR_MOTOR_MODE_KD].f_MAX		= MAX_MOTOR_MODE_KD;

	// initialize the int_reg_config array
	int_reg_config[ADDR_PHASE_ORDER].name			= NAME_PHASE_ORDER;
	int_reg_config[ADDR_PHASE_ORDER].cmd			= CMD_PHASE_ORDER;
	int_reg_config[ADDR_PHASE_ORDER].i_MIN			= MIN_PHASE_ORDER;
	int_reg_config[ADDR_PHASE_ORDER].i_MAX			= MAX_PHASE_ORDER;

	int_reg_config[ADDR_CAN_ID].name				= NAME_CAN_ID;
	int_reg_config[ADDR_CAN_ID].cmd					= CMD_CAN_ID;
	int_reg_config[ADDR_CAN_ID].i_MIN				= MIN_CAN_ID;
	int_reg_config[ADDR_CAN_ID].i_MAX				= MAX_CAN_ID;

	int_reg_config[ADDR_CAN_MASTER].name			= NAME_CAN_MASTER;
	int_reg_config[ADDR_CAN_MASTER].cmd				= CMD_CAN_MASTER;
	int_reg_config[ADDR_CAN_MASTER].i_MIN			= MIN_CAN_MASTER;
	int_reg_config[ADDR_CAN_MASTER].i_MAX			= MAX_CAN_MASTER;

	int_reg_config[ADDR_CAN_TIMEOUT].name			= NAME_CAN_TIMEOUT;
	int_reg_config[ADDR_CAN_TIMEOUT].cmd			= CMD_CAN_TIMEOUT;
	int_reg_config[ADDR_CAN_TIMEOUT].i_MIN			= MIN_CAN_TIMEOUT;
	int_reg_config[ADDR_CAN_TIMEOUT].i_MAX			= MAX_CAN_TIMEOUT;

	int_reg_config[ADDR_M_ZERO].name				= NAME_M_ZERO;
	int_reg_config[ADDR_M_ZERO].cmd					= CMD_M_ZERO;
	int_reg_config[ADDR_M_ZERO].i_MIN				= MIN_M_ZERO;
	int_reg_config[ADDR_M_ZERO].i_MAX				= MAX_M_ZERO;

	int_reg_config[ADDR_E_ZERO].name				= NAME_E_ZERO;
	int_reg_config[ADDR_E_ZERO].cmd					= CMD_E_ZERO;
	int_reg_config[ADDR_E_ZERO].i_MIN				= MIN_E_ZERO;
	int_reg_config[ADDR_E_ZERO].i_MAX				= MAX_E_ZERO;

	int_reg_config[ADDR_HALL_CAL_DIR].name			= NAME_HALL_CAL_DIR;
	int_reg_config[ADDR_HALL_CAL_DIR].cmd			= CMD_HALL_CAL_DIR;
	int_reg_config[ADDR_HALL_CAL_DIR].i_MIN			= MIN_HALL_CAL_DIR;
	int_reg_config[ADDR_HALL_CAL_DIR].i_MAX			= MAX_HALL_CAL_DIR;

	int_reg_config[ADDR_ENCODER_LUT].name			= NAME_ENCODER_LUT;
	int_reg_config[ADDR_ENCODER_LUT].cmd			= CMD_ENCODER_LUT;
	int_reg_config[ADDR_ENCODER_LUT].i_MIN			= MIN_ENCODER_LUT;
	int_reg_config[ADDR_ENCODER_LUT].i_MAX			= MAX_ENCODER_LUT;
}

char* float_reg_update_uart(char cmd, const char *c_data){
    static char response[100];
	float f_data = atof(c_data);

	if (!isfinite(f_data)){
		return STR_INVALID_VALUE;
	}

	for (int i=0; i<FLOAT_REG_LENGTH; i++){
		if (float_reg_config[i].cmd == cmd){
			if ((float_reg_config[i].f_MIN > f_data) || (float_reg_config[i].f_MAX < f_data)){
				return STR_INVALID_VALUE;
			}
			else if (i == ADDR_V_MAX){
				__float_reg[ADDR_V_MAX] = f_data;
				__float_reg[ADDR_V_MIN] = -f_data;
                sprintf(response, "%s set to %f\r\n", float_reg_config[i].name, f_data);
				return response;
			}
			else if (i == ADDR_T_MAX){
				__float_reg[ADDR_T_MAX] = f_data;
				__float_reg[ADDR_T_MIN] = -f_data;
                sprintf(response, "%s set to %f\r\n", float_reg_config[i].name, f_data);
				return response;
			}
			else{
				__float_reg[i] = f_data;
                sprintf(response, "%s set to %f\r\n", float_reg_config[i].name, f_data);
				return response;
			}
		}
	}
	return STR_INVALID_CMD;
}

int float_reg_update_can(int addr, float f_data){
	if (addr < 0 || addr >= FLOAT_REG_LENGTH){
		return CODE_INVALID_ADDR;
	}
	else if (!isfinite(f_data)){
		return CODE_INVALID_VALUE;
	}
	else if (float_reg_config[addr].cmd == ' '){
		return CODE_READ_ONLY;
	}
	else if ((float_reg_config[addr].f_MIN > f_data) || (float_reg_config[addr].f_MAX < f_data)){
		return CODE_INVALID_VALUE;
	}
	else{
		if (__float_reg[addr] != f_data){
			__float_reg[addr] = f_data;
			if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
			preference_writer_flush(&prefs);
			preference_writer_close(&prefs);
			preference_writer_load(prefs);
		}
		return CODE_CONFIG_SUCCESS;
	}
}

char* int_reg_update_uart(char cmd, const char *c_data){
    static char response[100];
	int i_data = atoi(c_data);

	for (int i=0; i<INT_REG_LENGTH; i++){
		if (int_reg_config[i].cmd == cmd){
			if ((int_reg_config[i].i_MIN > i_data) || (int_reg_config[i].i_MAX < i_data)){
				return STR_INVALID_VALUE;
			}
			else{
				__int_reg[i] = i_data;
                sprintf(response, "%s set to %d\r\n", int_reg_config[i].name, i_data);
				return response;
			}
		}
	}
	return STR_INVALID_CMD;
}

int int_reg_update_can(int addr, int i_data){
	if (addr < 0 || addr >= INT_REG_LENGTH){
		return CODE_INVALID_ADDR;
	}
	else if (int_reg_config[addr].cmd == ' '){
		return CODE_READ_ONLY;
	}
	else if (((int_reg_config[addr].i_MIN > i_data) || (int_reg_config[addr].i_MAX < i_data)) ||
			((addr == ADDR_HALL_CAL_DIR) && (i_data == 0))){
		return CODE_INVALID_VALUE;
	}
	else{
		if (__int_reg[addr] != i_data){
			__int_reg[addr] = i_data;
			if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
			preference_writer_flush(&prefs);
			preference_writer_close(&prefs);
			preference_writer_load(prefs);
		}
		return CODE_CONFIG_SUCCESS;
	}
}
