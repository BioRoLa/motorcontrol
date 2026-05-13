/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#define GLOBAL_MAX_VALUE		2147483647
#define GLOBAL_MIN_VALUE		-2147483647


#define FLOAT_REG_LENGTH		64
#define INT_REG_LENGTH			256


#define NAME_I_BW				"I_BW"         			            // Current loop bandwidth
#define ADDR_I_BW               2                                   // Current loop bandwidth
#define CMD_I_BW               	'b'                                 // Current loop bandwidth
#define MIN_I_BW               	100.0f                              // Current loop bandwidth
#define MAX_I_BW               	2000.0f                             // Current loop bandwidth

#define NAME_I_MAX              "I_MAX"                             // Current limit
#define ADDR_I_MAX              3                                   // Current limit
#define CMD_I_MAX              	'l'                                 // Current limit
#define MIN_I_MAX              	0.0f                                // Current limit
#define MAX_I_MAX              	75.0f                               // Current limit

#define NAME_THETA_MIN          "THETA_MIN"                         // Minimum position setpoint
#define ADDR_THETA_MIN          4                                   // Minimum position setpoint
#define CMD_THETA_MIN           ' '                                 // Minimum position setpoint
#define MIN_THETA_MIN           GLOBAL_MIN_VALUE                    // Minimum position setpoint
#define MAX_THETA_MIN           GLOBAL_MAX_VALUE                    // Minimum position setpoint

#define NAME_THETA_MAX          "THETA_MAX"                         // Maximum position setpoint
#define ADDR_THETA_MAX          5                                   // Maximum position setpoint
#define CMD_THETA_MAX           ' '                                 // Minimum position setpoint
#define MIN_THETA_MAX           GLOBAL_MIN_VALUE                    // Minimum position setpoint
#define MAX_THETA_MAX           GLOBAL_MAX_VALUE                    // Minimum position setpoint

#define NAME_I_FW_MAX           "I_FW_MAX"                          // Maximum field weakening current
#define ADDR_I_FW_MAX           6                                   // Maximum field weakening current
#define CMD_I_FW_MAX           	'f'                                 // Maximum field weakening current
#define MIN_I_FW_MAX           	0.0f                                // Maximum field weakening current
#define MAX_I_FW_MAX           	33.0f                               // Maximum field weakening current

#define NAME_R_NOMINAL          "R_NOMINAL"                         // Nominal motor resistance, set during calibration
#define ADDR_R_NOMINAL          7                                   // Nominal motor resistance, set during calibration
#define CMD_R_NOMINAL          	' '                                 // Nominal motor resistance, set during calibration
#define MIN_R_NOMINAL          	GLOBAL_MIN_VALUE                    // Nominal motor resistance, set during calibration
#define MAX_R_NOMINAL          	GLOBAL_MAX_VALUE                    // Nominal motor resistance, set during calibration

#define NAME_TEMP_MAX          	"TEMP_MAX"                          // Temperature safety lmit
#define ADDR_TEMP_MAX          	8                                   // Temperature safety lmit
#define CMD_TEMP_MAX           	' '                                 // Temperature safety lmit
#define MIN_TEMP_MAX           	GLOBAL_MIN_VALUE                    // Temperature safety lmit
#define MAX_TEMP_MAX           	GLOBAL_MAX_VALUE                    // Temperature safety lmit

#define NAME_I_MAX_CONT        	"I_MAX_CONT"                        // Continuous max current
#define ADDR_I_MAX_CONT        	9                                   // Continuous max current
#define CMD_I_MAX_CONT         	'c'                                 // Continuous max current
#define MIN_I_MAX_CONT         	GLOBAL_MIN_VALUE                    // Continuous max current
#define MAX_I_MAX_CONT         	GLOBAL_MAX_VALUE                    // Continuous max current

#define NAME_PPAIRS			   	"PPAIRS"							// Number of motor pole-pairs
#define ADDR_PPAIRS			   	10									// Number of motor pole-pairs
#define CMD_PPAIRS			   	' '									// Number of motor pole-pairs
#define MIN_PPAIRS			   	GLOBAL_MIN_VALUE					// Number of motor pole-pairs
#define MAX_PPAIRS			   	GLOBAL_MAX_VALUE					// Number of motor pole-pairs

//#define NAME_L_D			   	"L_D"								// D-axis inductance
//#define ADDR_L_D			   	11									// D-axis inductance
//#define CMD_L_D			   	' '									// D-axis inductance
//#define MIN_L_D			   	GLOBAL_MIN_VALUE					// D-axis inductance
//#define MAX_L_D			  	GLOBAL_MAX_VALUE					// D-axis inductance

//#define NAME_L_Q				"L_Q"								// Q-axis inductance
//#define ADDR_L_Q				12									// Q-axis inductance
//#define CMD_L_Q				' '									// Q-axis inductance
//#define MIN_L_Q				GLOBAL_MIN_VALUE					// Q-axis inductance
//#define MAX_L_Q				GLOBAL_MAX_VALUE					// Q-axis inductance

#define NAME_R_PHASE			"R_PHASE"							// Single phase resistance
#define ADDR_R_PHASE			13									// Single phase resistance
#define CMD_R_PHASE				' '									// Single phase resistance
#define MIN_R_PHASE				GLOBAL_MIN_VALUE					// Single phase resistance
#define MAX_R_PHASE				GLOBAL_MAX_VALUE					// Single phase resistance

#define NAME_KT					"KT"								// Torque Constant (N-m/A)
#define ADDR_KT					14									// Torque Constant (N-m/A)
#define CMD_KT				   	't'									// Torque Constant (N-m/A)
#define MIN_KT					0.0001f								// Torque Constant (N-m/A)
#define MAX_KT					GLOBAL_MAX_VALUE					// Torque Constant (N-m/A)

#define NAME_R_TH				"R_TH"								// Thermal resistance (C/W)
#define ADDR_R_TH				15									// Thermal resistance (C/W)
#define CMD_R_TH				' '									// Thermal resistance (C/W)
#define MIN_R_TH				GLOBAL_MIN_VALUE					// Thermal resistance (C/W)
#define MAX_R_TH				GLOBAL_MAX_VALUE					// Thermal resistance (C/W)

#define NAME_C_TH				"C_TH"								// Thermal mass (C/J)
#define ADDR_C_TH				16									// Thermal mass (C/J)
#define CMD_C_TH				' '									// Thermal mass (C/J)
#define MIN_C_TH				GLOBAL_MIN_VALUE					// Thermal mass (C/J)
#define MAX_C_TH				GLOBAL_MAX_VALUE					// Thermal mass (C/J)

#define NAME_GR					"GR"								// Gear ratio
#define ADDR_GR					17									// Gear ratio
#define CMD_GR					'g'									// Gear ratio
#define MIN_GR					0.001f								// Gear ratio
#define MAX_GR					GLOBAL_MAX_VALUE					// Gear ratio

#define NAME_I_CAL				"I_CAL"								// Calibration Current
#define ADDR_I_CAL				18									// Calibration Current
#define CMD_I_CAL				'a'									// Calibration Current
#define MIN_I_CAL				0.0f								// Calibration Current
#define MAX_I_CAL				20.0f								// Calibration Current

#define NAME_P_MIN				"P_MIN"								// Position setpoint lower limit (rad)
#define ADDR_P_MIN				19									// Position setpoint lower limit (rad)
#define CMD_P_MIN				'_'									// Position setpoint lower limit (rad)
#define MIN_P_MIN				GLOBAL_MIN_VALUE								// Position setpoint lower limit (rad)
#define MAX_P_MIN				0.0f								// Position setpoint lower limit (rad)

#define NAME_P_MAX				"P_MAX"								// Position setupoint upper bound (rad)
#define ADDR_P_MAX				20									// Position setupoint upper bound (rad)
#define CMD_P_MAX				'p'									// Position setupoint upper bound (rad)
#define MIN_P_MAX				0.0f								// Position setupoint upper bound (rad)
#define MAX_P_MAX				GLOBAL_MAX_VALUE					// Position setupoint upper bound (rad)

#define NAME_V_MIN				"V_MIN"								// Velocity setpoint lower bound (rad/s)
#define ADDR_V_MIN				21									// Velocity setpoint lower bound (rad/s)
#define CMD_V_MIN				'_'									// Velocity setpoint lower bound (rad/s)
#define MIN_V_MIN				GLOBAL_MIN_VALUE					// Velocity setpoint lower bound (rad/s)
#define MAX_V_MIN				0.0f								// Velocity setpoint lower bound (rad/s)

#define NAME_V_MAX				"V_MAX"								// Velocity setpoint upper bound (rad/s)
#define ADDR_V_MAX				22									// Velocity setpoint upper bound (rad/s)
#define CMD_V_MAX				'v'									// Velocity setpoint upper bound (rad/s)
#define MIN_V_MAX				0.0f								// Velocity setpoint upper bound (rad/s)
#define MAX_V_MAX				GLOBAL_MAX_VALUE					// Velocity setpoint upper bound (rad/s)

#define NAME_T_MIN     			"T_MIN"
#define ADDR_T_MIN			    23
#define CMD_T_MIN		     	'_'
#define MIN_T_MIN		     	GLOBAL_MIN_VALUE
#define MAX_T_MIN		     	0.0f

#define NAME_T_MAX     			"T_MAX"
#define ADDR_T_MAX			    24
#define CMD_T_MAX		     	'T'
#define MIN_T_MAX		     	0.0f
#define MAX_T_MAX		     	GLOBAL_MAX_VALUE

#define NAME_KP_MAX				"KP_MAX"							// Max position gain (N-m/rad)
#define ADDR_KP_MAX				25									// Max position gain (N-m/rad)
#define CMD_KP_MAX				'k'									// Max position gain (N-m/rad)
#define MIN_KP_MAX				0.0f								// Max position gain (N-m/rad)
#define MAX_KP_MAX				GLOBAL_MAX_VALUE					// Max position gain (N-m/rad)

#define NAME_KI_MAX				"KI_MAX"
#define ADDR_KI_MAX				26
#define CMD_KI_MAX				'i'
#define MIN_KI_MAX				0.0f
#define MAX_KI_MAX				GLOBAL_MAX_VALUE

#define NAME_KD_MAX				"KD_MAX"							// Max velocity gain (N-m/rad/s)
#define ADDR_KD_MAX				27									// Max velocity gain (N-m/rad/s)
#define CMD_KD_MAX				'd'									// Max velocity gain (N-m/rad/s)
#define MIN_KD_MAX				0.0f								// Max velocity gain (N-m/rad/s)
#define MAX_KD_MAX				GLOBAL_MAX_VALUE					// Max velocity gain (N-m/rad/s)

#define NAME_HALL_CAL_OFFSET    "HALL_CAL_OFFSET"
#define ADDR_HALL_CAL_OFFSET    28
#define CMD_HALL_CAL_OFFSET    	'e'
#define MIN_HALL_CAL_OFFSET    	0.0f
#define MAX_HALL_CAL_OFFSET    	143.0f

#define NAME_HALL_CAL_SPEED     "HALL_CAL_SPEED"
#define ADDR_HALL_CAL_SPEED     29
#define CMD_HALL_CAL_SPEED     	'h'
#define MIN_HALL_CAL_SPEED     	0.0f
#define MAX_HALL_CAL_SPEED     	10.0f

#define NAME_HALL_CAL_KP        "HALL_CAL_KP"
#define ADDR_HALL_CAL_KP        30
#define CMD_HALL_CAL_KP         'K'
#define MIN_HALL_CAL_KP         0.0f
#define MAX_HALL_CAL_KP         1000.0f

#define NAME_HALL_CAL_KI        "HALL_CAL_KI"
#define ADDR_HALL_CAL_KI        31
#define CMD_HALL_CAL_KI         'I'
#define MIN_HALL_CAL_KI         0.0f
#define MAX_HALL_CAL_KI         10.0f

#define NAME_HALL_CAL_KD        "HALL_CAL_KD"
#define ADDR_HALL_CAL_KD        32
#define CMD_HALL_CAL_KD         'D'
#define MIN_HALL_CAL_KD         0.0f
#define MAX_HALL_CAL_KD         5.0f

#define NAME_MOTOR_MODE_KP       "MOTOR_MODE_KP"
#define ADDR_MOTOR_MODE_KP       33
#define CMD_MOTOR_MODE_KP        'X'
#define MIN_MOTOR_MODE_KP        0.0f
#define MAX_MOTOR_MODE_KP        1000.0f

#define NAME_MOTOR_MODE_KI       "MOTOR_MODE_KI"
#define ADDR_MOTOR_MODE_KI       34
#define CMD_MOTOR_MODE_KI        'Y'
#define MIN_MOTOR_MODE_KI        0.0f
#define MAX_MOTOR_MODE_KI        10.0f

#define NAME_MOTOR_MODE_KD       "MOTOR_MODE_KD"
#define ADDR_MOTOR_MODE_KD       35
#define CMD_MOTOR_MODE_KD        'Z'
#define MIN_MOTOR_MODE_KD        0.0f
#define MAX_MOTOR_MODE_KD        5.0f

#define NAME_ABAD_CAL_OFFSET    "ABAD_CAL_OFFSET"
#define ADDR_ABAD_CAL_OFFSET    36
#define CMD_ABAD_CAL_OFFSET    	'O'
#define MIN_ABAD_CAL_OFFSET    	0.0f
#define MAX_ABAD_CAL_OFFSET    	90.0f

#define NAME_ABAD_CAL_SPEED     "ABAD_CAL_SPEED"
#define ADDR_ABAD_CAL_SPEED     37
#define CMD_ABAD_CAL_SPEED     	'S'
#define MIN_ABAD_CAL_SPEED     	0.0f
#define MAX_ABAD_CAL_SPEED     	10.0f

#define NAME_ABAD_CAL_KP        "ABAD_CAL_KP"
#define ADDR_ABAD_CAL_KP        38
#define CMD_ABAD_CAL_KP         'J'
#define MIN_ABAD_CAL_KP         0.0f
#define MAX_ABAD_CAL_KP         1000.0f

#define NAME_ABAD_CAL_KI        "ABAD_CAL_KI"
#define ADDR_ABAD_CAL_KI        39
#define CMD_ABAD_CAL_KI         'U'
#define MIN_ABAD_CAL_KI         0.0f
#define MAX_ABAD_CAL_KI         10.0f

#define NAME_ABAD_CAL_KD        "ABAD_CAL_KD"
#define ADDR_ABAD_CAL_KD        40
#define CMD_ABAD_CAL_KD         'V'
#define MIN_ABAD_CAL_KD         0.0f
#define MAX_ABAD_CAL_KD         5.0f


#define I_BW                    __float_reg[ADDR_I_BW]	            // Current loop bandwidth
#define I_MAX                   __float_reg[ADDR_I_MAX]             // Current limit
#define THETA_MIN               __float_reg[ADDR_THETA_MIN]         // Minimum position setpoint
#define THETA_MAX               __float_reg[ADDR_THETA_MAX]         // Maximum position setpoint
#define I_FW_MAX                __float_reg[ADDR_I_FW_MAX]          // Maximum field weakening current
#define R_NOMINAL               __float_reg[ADDR_R_NOMINAL]         // Nominal motor resistance, set during calibration
#define TEMP_MAX                __float_reg[ADDR_TEMP_MAX]          // Temperature safety lmit
#define I_MAX_CONT              __float_reg[ADDR_I_MAX_CONT]        // Continuous max current
#define PPAIRS					__float_reg[ADDR_PPAIRS]			// Number of motor pole-pairs
//#define L_D						__float_reg[ADDR_L_D]			// D-axis inductance
//#define L_Q						__float_reg[ADDR_L_Q]			// Q-axis inductance
#define R_PHASE					__float_reg[ADDR_R_PHASE]			// Single phase resistance
#define KT						__float_reg[ADDR_KT]				// Torque Constant (N-m/A)
#define R_TH					__float_reg[ADDR_R_TH]				// Thermal resistance (C/W)
#define C_TH					__float_reg[ADDR_C_TH]				// Thermal mass (C/J)
#define GR						__float_reg[ADDR_GR]				// Gear ratio
#define I_CAL					__float_reg[ADDR_I_CAL]				// Calibration Current
#define P_MIN					__float_reg[ADDR_P_MIN]				// Position setpoint lower limit (rad)
#define P_MAX					__float_reg[ADDR_P_MAX]				// Position setupoint upper bound (rad)
#define V_MIN					__float_reg[ADDR_V_MIN]				// Velocity setpoint lower bound (rad/s)
#define V_MAX					__float_reg[ADDR_V_MAX]				// Velocity setpoint upper bound (rad/s)
#define T_MIN			        __float_reg[ADDR_T_MIN]
#define T_MAX			        __float_reg[ADDR_T_MAX]
#define KP_MAX					__float_reg[ADDR_KP_MAX]			// Max position gain (N-m/rad)
#define KI_MAX					__float_reg[ADDR_KI_MAX]
#define KD_MAX					__float_reg[ADDR_KD_MAX]			// Max velocity gain (N-m/rad/s)
#define HALL_CAL_OFFSET         __float_reg[ADDR_HALL_CAL_OFFSET]
#define HALL_CAL_SPEED          __float_reg[ADDR_HALL_CAL_SPEED]
#define HALL_CAL_KP             __float_reg[ADDR_HALL_CAL_KP]
#define HALL_CAL_KI             __float_reg[ADDR_HALL_CAL_KI]
#define HALL_CAL_KD             __float_reg[ADDR_HALL_CAL_KD]
#define MOTOR_MODE_KP           __float_reg[ADDR_MOTOR_MODE_KP]
#define MOTOR_MODE_KI           __float_reg[ADDR_MOTOR_MODE_KI]
#define MOTOR_MODE_KD           __float_reg[ADDR_MOTOR_MODE_KD]
#define ABAD_CAL_OFFSET         __float_reg[ADDR_ABAD_CAL_OFFSET]
#define ABAD_CAL_SPEED          __float_reg[ADDR_ABAD_CAL_SPEED]
#define ABAD_CAL_KP             __float_reg[ADDR_ABAD_CAL_KP]
#define ABAD_CAL_KI             __float_reg[ADDR_ABAD_CAL_KI]
#define ABAD_CAL_KD             __float_reg[ADDR_ABAD_CAL_KD]


#define NAME_PHASE_ORDER        "PHASE_ORDER"	                    // Phase swapping during calibration
#define ADDR_PHASE_ORDER        0				                    // Phase swapping during calibration
#define CMD_PHASE_ORDER        	' '				                    // Phase swapping during calibration
#define MIN_PHASE_ORDER        	GLOBAL_MIN_VALUE                    // Phase swapping during calibration
#define MAX_PHASE_ORDER        	GLOBAL_MAX_VALUE                    // Phase swapping during calibration

#define NAME_CAN_ID             "CAN_ID"	                        // CAN bus ID
#define ADDR_CAN_ID             1			                        // CAN bus ID
#define CMD_CAN_ID             	'n'			                        // CAN bus ID
#define MIN_CAN_ID             	1			                        // CAN bus ID
#define MAX_CAN_ID             	127			                        // CAN bus ID

#define NAME_CAN_MASTER         "CAN_MASTER"                        // CAN bus "master" ID
#define ADDR_CAN_MASTER         2			                        // CAN bus "master" ID
#define CMD_CAN_MASTER         	'm'			                        // CAN bus "master" ID
#define MIN_CAN_MASTER         	0			                        // CAN bus "master" ID
#define MAX_CAN_MASTER         	127			                        // CAN bus "master" ID

#define NAME_CAN_TIMEOUT        "CAN_TIMEOUT"                       // CAN bus timeout period
#define ADDR_CAN_TIMEOUT        3			                        // CAN bus timeout period
#define CMD_CAN_TIMEOUT        	'o'			                        // CAN bus timeout period
#define MIN_CAN_TIMEOUT        	0			                        // CAN bus timeout period
#define MAX_CAN_TIMEOUT        	100000			                    // CAN bus timeout period

#define NAME_M_ZERO				"M_ZERO"
#define ADDR_M_ZERO				4
#define CMD_M_ZERO				' '
#define MIN_M_ZERO				GLOBAL_MIN_VALUE
#define MAX_M_ZERO				GLOBAL_MAX_VALUE

#define NAME_E_ZERO				"E_ZERO"
#define ADDR_E_ZERO				5
#define CMD_E_ZERO				' '
#define MIN_E_ZERO				GLOBAL_MIN_VALUE
#define MAX_E_ZERO				GLOBAL_MAX_VALUE

#define NAME_HALL_CAL_DIR       "HALL_CAL_DIR"
#define ADDR_HALL_CAL_DIR       6
#define CMD_HALL_CAL_DIR       	'r'
#define MIN_HALL_CAL_DIR       	-1
#define MAX_HALL_CAL_DIR       	1

#define NAME_ABAD_CAL_DIR       "ABAD_CAL_DIR"
#define ADDR_ABAD_CAL_DIR       7
#define CMD_ABAD_CAL_DIR       	'R'
#define MIN_ABAD_CAL_DIR       	-1
#define MAX_ABAD_CAL_DIR       	1

#define NAME_ENCODER_LUT        "ENCODER_LUT"
#define ADDR_ENCODER_LUT        8
#define CMD_ENCODER_LUT        	' '
#define MIN_ENCODER_LUT        	GLOBAL_MIN_VALUE
#define MAX_ENCODER_LUT        	GLOBAL_MAX_VALUE

#define NAME_MOTOR_POSITION     "MOTOR_POSITION"
#define ADDR_MOTOR_POSITION     9
#define CMD_MOTOR_POSITION     	'P'
#define MIN_MOTOR_POSITION     	0
#define MAX_MOTOR_POSITION     	2

/* Motor position/role definitions */
#define MOTOR_POS_HIP           0  // Hip flexion/extension
#define MOTOR_POS_ABAD_FL_RR    1  // AB/AD for Front-Left or Rear-Right
#define MOTOR_POS_ABAD_FR_RL    2  // AB/AD for Front-Right or Rear-Left


#define PHASE_ORDER             __int_reg[ADDR_PHASE_ORDER]         // Phase swapping during calibration
#define CAN_ID                  __int_reg[ADDR_CAN_ID]              // CAN bus ID
#define CAN_MASTER              __int_reg[ADDR_CAN_MASTER]          // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[ADDR_CAN_TIMEOUT]         // CAN bus timeout period
#define M_ZERO					__int_reg[ADDR_M_ZERO]
#define E_ZERO					__int_reg[ADDR_E_ZERO]
#define HALL_CAL_DIR            __int_reg[ADDR_HALL_CAL_DIR]
#define ABAD_CAL_DIR            __int_reg[ADDR_ABAD_CAL_DIR]
#define ENCODER_LUT             __int_reg[ADDR_ENCODER_LUT]         // Encoder offset LUT - 128 elements long
#define MOTOR_POSITION          __int_reg[ADDR_MOTOR_POSITION]      // Motor position/role (HIP, ABAD_FL/RR, ABAD_FR/RL)


#define STR_INVALID_VALUE		"Not a valid value\r\n"
#define STR_INVALID_CMD			"Not a valid command\r\n"
#define CODE_CONFIG_SUCCESS		0
#define CODE_INVALID_VALUE		1
#define CODE_READ_ONLY			2
#define CODE_INVALID_ADDR		3
#define CODE_INVALID_CMD		4

#define CODE_HALL_UNCALIBRATED	0
#define CODE_HALL_CALIBRATING	1
#define CODE_HALL_CAL_SUCCESS	2
#define CODE_HALL_CAL_FAIL		3

#define CODE_ABAD_UNCALIBRATED	0
#define CODE_ABAD_CALIBRATING	1
#define CODE_ABAD_CAL_SUCCESS	2
#define CODE_ABAD_CAL_FAIL		3


struct FloatRegConfig{
	char *name;
	char cmd;
	float f_MIN;
	float f_MAX;
};

struct IntRegConfig{
	char *name;
	char cmd;
	int i_MIN;
	int i_MAX;
};


extern float __float_reg[];
extern int __int_reg[];


void user_config_initialize(void);
char* float_reg_update_uart(char cmd, const char *c_data);
int float_reg_update_can(int addr, float f_data);
char* int_reg_update_uart(char cmd, const char *c_data);
int int_reg_update_can(int addr, int i_data);

#ifdef __cplusplus
}
#endif

#endif
