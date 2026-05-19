/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
#include "abad_calibration.h"
#include "version_info.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//#define VERSION_NUM 2.0f -> defined in main.h (because CAN pack_reply needs to use)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[FLOAT_REG_LENGTH];
int __int_reg[INT_REG_LENGTH];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;
HallCalStruct hall_cal;
HallCalStructABAD abad_cal;

/* init but don't allocate calibration arrays */
int *error_array = NULL;
int *lut_array = NULL;

uint8_t Serial2RxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  /* Load settings from flash */
  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);

  /* Sanitize configs in case flash is empty or is wrong value*/
  if(                          MIN_E_ZERO > E_ZERO                   || MAX_E_ZERO < E_ZERO)                   {E_ZERO = 0;}
  if(                          MIN_M_ZERO > M_ZERO                   || MAX_M_ZERO < M_ZERO)                   {M_ZERO = 0;}
  if(isnan(GR)              || MIN_GR > GR		                     || MAX_GR < GR)                 		   {GR = 6.0f;}
  if(isnan(KT)	            || MIN_KT > KT                           || MAX_KT < KT)                           {KT = 0.08f;}
  if(isnan(I_BW)            || MIN_I_BW > I_BW                       || MAX_I_BW < I_BW)				       {I_BW = 1000;}
  if(isnan(I_MAX)           || MIN_I_MAX > I_MAX                     || MAX_I_MAX < I_MAX)					   {I_MAX = 40;}
  if(isnan(P_MAX)           || MIN_P_MAX > P_MAX                     || MAX_P_MAX < P_MAX)	                   {P_MAX = 6.283f;}
  if(isnan(P_MIN)           || MIN_P_MIN > P_MIN                     || MAX_P_MIN < P_MIN)	                   {P_MIN = 0.0f;}
  if(isnan(V_MAX)           || MIN_V_MAX > V_MAX                     || MAX_V_MAX < V_MAX)	                   {V_MAX = 45.0f;}
  if(isnan(V_MIN)           || MIN_V_MIN > V_MIN                     || MAX_V_MIN < V_MIN)	                   {V_MIN = -45.0f;}
  if(isnan(T_MAX)           || MIN_T_MAX > T_MAX                     || MAX_T_MAX < T_MAX)                     {T_MAX = 20.0f;}
  if(isnan(T_MIN)           || MIN_T_MIN > T_MIN                     || MAX_T_MIN < T_MIN)	                   {T_MIN = -20.0f;}
  if(isnan(KP_MAX)          || MIN_KP_MAX > KP_MAX                   || MAX_KP_MAX < KP_MAX)			       {KP_MAX = 500.0f;}
  if(isnan(KI_MAX)          || MIN_KI_MAX > KI_MAX                   || MAX_KI_MAX < KI_MAX)      			   {KI_MAX = 0.0f;}
  if(isnan(KD_MAX)          || MIN_KD_MAX > KD_MAX                   || MAX_KD_MAX < KD_MAX)				   {KD_MAX = 5.0f;}
  if(isnan(I_FW_MAX)        || MIN_I_FW_MAX > I_FW_MAX               || MAX_I_FW_MAX < I_FW_MAX)			   {I_FW_MAX = 0.0f;}
  if(isnan(I_MAX_CONT)      || MIN_I_MAX_CONT > I_MAX_CONT           || MAX_I_MAX_CONT < I_MAX_CONT)           {I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)           || MIN_I_CAL > I_CAL                     || MAX_I_CAL < I_CAL)					   {I_CAL = 5.0f;}
  if(HALL_CAL_DIR != 1 && HALL_CAL_DIR != -1)												      			   {HALL_CAL_DIR = 1;}
  if(isnan(HALL_CAL_OFFSET) || MIN_HALL_CAL_OFFSET > HALL_CAL_OFFSET || MAX_HALL_CAL_OFFSET < HALL_CAL_OFFSET) {HALL_CAL_OFFSET = 0.0f;}
  if(isnan(HALL_CAL_SPEED)  || MIN_HALL_CAL_SPEED > HALL_CAL_SPEED   || MAX_HALL_CAL_SPEED < HALL_CAL_SPEED)   {HALL_CAL_SPEED = 0.25f;}
  if(						   MIN_CAN_ID > CAN_ID                   || MAX_CAN_ID < CAN_ID)                   {CAN_ID = 1;}
  if(isnan(HALL_CAL_KP)     || MIN_HALL_CAL_KP > HALL_CAL_KP         || MAX_HALL_CAL_KP < HALL_CAL_KP)         {HALL_CAL_KP = 50.0f;}
  if(isnan(HALL_CAL_KI)     || MIN_HALL_CAL_KI > HALL_CAL_KI         || MAX_HALL_CAL_KI < HALL_CAL_KI)         {HALL_CAL_KI = 0.0f;}
  if(isnan(HALL_CAL_KD)     || MIN_HALL_CAL_KD > HALL_CAL_KD         || MAX_HALL_CAL_KD < HALL_CAL_KD)         {HALL_CAL_KD = 1.5f;}
  if(ABAD_CAL_DIR != 1 && ABAD_CAL_DIR != -1)                                                                             {ABAD_CAL_DIR = 1;}
  if(isnan(ABAD_CAL_OFFSET) || MIN_ABAD_CAL_OFFSET > ABAD_CAL_OFFSET || MAX_ABAD_CAL_OFFSET < ABAD_CAL_OFFSET)          {ABAD_CAL_OFFSET = 0.0f;}
  if(isnan(ABAD_CAL_SPEED)  || MIN_ABAD_CAL_SPEED > ABAD_CAL_SPEED   || MAX_ABAD_CAL_SPEED < ABAD_CAL_SPEED)            {ABAD_CAL_SPEED = 0.25f;}
  if(isnan(ABAD_CAL_KP)     || MIN_ABAD_CAL_KP > ABAD_CAL_KP         || MAX_ABAD_CAL_KP < ABAD_CAL_KP)                  {ABAD_CAL_KP = 50.0f;}
  if(isnan(ABAD_CAL_KI)     || MIN_ABAD_CAL_KI > ABAD_CAL_KI         || MAX_ABAD_CAL_KI < ABAD_CAL_KI)                  {ABAD_CAL_KI = 0.0f;}
  if(isnan(ABAD_CAL_KD)     || MIN_ABAD_CAL_KD > ABAD_CAL_KD         || MAX_ABAD_CAL_KD < ABAD_CAL_KD)                  {ABAD_CAL_KD = 1.5f;}
  if(isnan(MOTOR_MODE_KP)   || MIN_MOTOR_MODE_KP > MOTOR_MODE_KP     || MAX_MOTOR_MODE_KP < MOTOR_MODE_KP)     {MOTOR_MODE_KP = 5.0f;}
  if(isnan(MOTOR_MODE_KI)   || MIN_MOTOR_MODE_KI > MOTOR_MODE_KI     || MAX_MOTOR_MODE_KI < MOTOR_MODE_KI)     {MOTOR_MODE_KI = 0.0f;}
  if(isnan(MOTOR_MODE_KD)   || MIN_MOTOR_MODE_KD > MOTOR_MODE_KD     || MAX_MOTOR_MODE_KD < MOTOR_MODE_KD)     {MOTOR_MODE_KD = 1.0f;}
  if(                          MIN_CAN_MASTER > CAN_MASTER           || MAX_CAN_MASTER < CAN_MASTER)           {CAN_MASTER = 0;}
  if(                          MIN_CAN_TIMEOUT > CAN_TIMEOUT         || MAX_CAN_TIMEOUT < CAN_TIMEOUT)         {CAN_TIMEOUT = 0;}
  if(isnan(R_NOMINAL)       || MIN_R_NOMINAL > R_NOMINAL             || MAX_R_NOMINAL < R_NOMINAL)             {R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX)        || MIN_TEMP_MAX > TEMP_MAX               || MAX_TEMP_MAX < TEMP_MAX)               {TEMP_MAX = 125.0f;}
  if(isnan(PPAIRS)          || MIN_PPAIRS > PPAIRS                   || MAX_PPAIRS < PPAIRS)                   {PPAIRS = 21.0f;}
  if(                          MIN_MOTOR_POSITION > MOTOR_POSITION   || MAX_MOTOR_POSITION < MOTOR_POSITION)   {MOTOR_POSITION = MOTOR_POS_HIP;}

  user_config_initialize();

//  printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);
  printf("\r\n= = = = = Version Information = = = = =\r\n");
  printf("\r\nFirmware Version:  %s\r\n", FIRMWARE_VERSION);
  printf("\r\nBuild Date:  %s\r\n", FIRMWARE_DATE);
  printf("Build Time:  %s\r\n", FIRMWARE_TIME);
  printf("\r\nAuthor:  %s\r\n", AUTHOR_NAME);
  printf("\r\nModification Info:  %s\r\n", MODIFICATION_INFO);
  printf("\r\n= = = = = Program Started = = = = =\r\n");

  /* Controller Setup */
  if(PHASE_ORDER){							// Timer channel to phase mapping

  }
  else{

  }

  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* initialize the parameters of hall sensor */
  hall_cal.hall_input = 1;
  hall_cal.hall_preinput = 1;
  hall_cal.hall_cal_pcmd = 0;
  hall_cal.hall_cal_speed = 0.25; // rad/s
  hall_cal.hall_present_pos = 0; // calibrate the previous position of encoder
  hall_cal.hall_in_pos = 0;  // read the position from 1 to 0 (magnet enters the area of hall sensor)
  hall_cal.hall_out_pos = 0; // read the position from 0 to 1 (magnet exits the area of hall sensor)
  hall_cal.hall_mid_pos = 0;
  hall_cal. hall_cal_offset = 0; // rad
  hall_cal.hall_cal_count = 0;
  hall_cal.hall_cal_state = CODE_HALL_UNCALIBRATED;

  /* AB/AD Hall calibration setup */
  abad_cal.hall_a_input = 1;
  abad_cal.hall_a_preinput = 1;
  abad_cal.hall_b_input = 1;
  abad_cal.hall_b_preinput = 1;
  abad_cal.abad_cal_pcmd = 0;
  abad_cal.abad_cal_speed = 0.25; // rad/s
  abad_cal.abad_present_pos = 0;
  abad_cal.bottom_transition_count = 0;
  abad_cal.abad_cal_state = CODE_ABAD_UNCALIBRATED;

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;

  if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
  else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  /* DRV8323 setup */
  DRV_CS_HIGH; 	// CS high
  GPIO_ENABLE;   // GPIO ENABLE_PIN HIGH
  HAL_Delay(1);
  drv_calibrate(drv);
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);
  zero_current(&controller);
  drv_enable_gd(drv);
  GPIO_DISABLE;
//  printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //start CAN
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt

  /* Set Interrupt Priorities */
  HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0); // commutation > communication
  HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;
  state.print_uart_msg = 0;
  enter_menu_state();


  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);
	 // drv_print_faults(drv);
	 // if(state.state==MOTOR_MODE){
	  	  //printf("%.2f %.2f %.2f %.2f %.2f\r\n", controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff);
	  //}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
