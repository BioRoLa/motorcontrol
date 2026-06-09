/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "structs.h"
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include <string.h>
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	msg->filter.FilterIdHigh=CAN_ID<<5;         // CAN ID
	msg->filter.FilterIdLow=0x0;
	msg->filter.FilterMaskIdHigh=0xFFF;
	msg->filter.FilterMaskIdLow=0xFFFF;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_16BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
}

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.DLC = 8; 			// message size of 8 byte
	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard
	msg->tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
	msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply_default(CANRxMessage rx_msg, CANTxMessage *tx_msg, float p, float v, float t, int version, int calibrate_finish, int state){
    int p_int = float_to_uint(p, P_REPLY_MIN, P_REPLY_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 16);
//    int t_int = float_to_uint(t, -(I_MAX+SENSE_BUFFER)*KT*GR, (I_MAX+SENSE_BUFFER)*KT*GR, 16);
    int t_int = float_to_uint(t, T_MIN, T_MAX, 16);

    tx_msg->data[0] = p_int>>8;
    tx_msg->data[1] = p_int&0xFF;
    tx_msg->data[2] = v_int>>8;
    tx_msg->data[3] = v_int&0xFF;
    tx_msg->data[4] = t_int>>8;
    tx_msg->data[5] = t_int&0xFF;
    tx_msg->data[6] = 0x0 + (calibrate_finish&0xF);
    tx_msg->data[7] = (version<<4) + (state&0xF);
}

void pack_reply_config(CANRxMessage rx_msg, CANTxMessage *tx_msg, int version, int state){
	int func_type = rx_msg.data[0];
	int reg_type = rx_msg.data[1];
	int target_addr = rx_msg.data[2];
	union RegData reg_data;
	int config_state;

	if (func_type == 1){
		if (state == MOTOR_MODE){
			config_state = CODE_READ_ONLY;
		}
		else{
		config_state = unpack_config_cmd(rx_msg);
		}
	}
	else if ((reg_type == 0 && (target_addr < 0 || target_addr >= INT_REG_LENGTH)) ||
			 (reg_type == 1 && (target_addr < 0 || target_addr >= FLOAT_REG_LENGTH))){
		config_state = CODE_INVALID_ADDR;
	}
	else if (reg_type != 0 && reg_type != 1){
		config_state = CODE_INVALID_CMD;
	}
	else{
		config_state = CODE_CONFIG_SUCCESS;
	}

	if (config_state != CODE_INVALID_ADDR){
		if (reg_type == 0 ){
	        reg_data.intValue = __int_reg[target_addr];
	        memcpy(&(tx_msg->data[3]), &(reg_data.intValue), sizeof(int));
		}
		else if (reg_type == 1){
            reg_data.floatValue = __float_reg[target_addr];
            memcpy(&(tx_msg->data[3]), &(reg_data.floatValue), sizeof(float));
		}
	}

    if (config_state == CODE_INVALID_ADDR || config_state == CODE_INVALID_CMD) {
        int zeroData = 0;
        memcpy(&(tx_msg->data[3]), &zeroData, sizeof(int));
    }

	tx_msg->data[0] = config_state;
	tx_msg->data[1] = reg_type;
	tx_msg->data[2] = target_addr;
	tx_msg->data[7] = (version<<4) + (state&0xF);
}

void pack_reply_hall_cal(CANRxMessage rx_msg, CANTxMessage *tx_msg, int version, int state){
	int config_state;

	if (rx_msg.data[0] == 1){
		config_state = unpack_hall_cal_cmd(rx_msg);
	}
	else{
		config_state = CODE_CONFIG_SUCCESS;
	}

	tx_msg->data[0] = config_state;
	tx_msg->data[1] = 0;
	tx_msg->data[2] = 0;

	memcpy((void*)&(tx_msg->data[3]), (void*)&__float_reg[ADDR_HALL_CAL_OFFSET], sizeof(float));

	tx_msg->data[7] = (version<<4) + (state&0xF);
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_control_cmd(CANRxMessage rx_msg, float *commands){// ControllerStruct * controller){
        int p_int = (rx_msg.data[0]<<8)|rx_msg.data[1];
        int kp_int = (rx_msg.data[2]<<4)|(rx_msg.data[3]>>4);
        int ki_int = ((rx_msg.data[3]&0xF)<<8)|rx_msg.data[4];
        int kd_int = (rx_msg.data[5]<<4)|(rx_msg.data[6]>>4);
        int t_int = ((rx_msg.data[6]&0xF)<<8)|rx_msg.data[7];

        commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        commands[1] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        commands[2] = uint_to_float(ki_int, KI_MIN, KI_MAX, 12);
        commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
}

int unpack_config_cmd(CANRxMessage rx_msg){
	int reg_type = rx_msg.data[1];
	int target_addr = rx_msg.data[2];
	int config_state;

	union RegData new_data;

	if (reg_type == 0){
		memcpy((void*)&(new_data.intValue), (void*)&(rx_msg.data[3]), sizeof(int));
		config_state = int_reg_update_can(target_addr, new_data.intValue);
	}
	else if (reg_type == 1){
		memcpy((void*)&(new_data.floatValue), (void*)&(rx_msg.data[3]), sizeof(float));
		config_state = float_reg_update_can(target_addr, new_data.floatValue);
	}
	else{
		config_state = CODE_INVALID_CMD;
	}

	return config_state;
}

int unpack_hall_cal_cmd(CANRxMessage rx_msg){
	float new_data = 0;
	int config_state;

	memcpy((void*)&new_data, (void*)&(rx_msg.data[1]), sizeof(float));

	config_state = float_reg_update_can(ADDR_HALL_CAL_OFFSET, new_data);

	return config_state;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
