# motorcontrol firmware

STM32F4 firmware for high-bandwidth 3-phase motor control, based on the integrated controller hardware used in the MIT Mini Cheetah ecosystem and related designs.

This repo targets STM32CubeIDE-generated projects and is designed for practical robot actuator control with:

- FOC current loop
- position/torque control
- encoder calibration
- single hall calibration
- AB/AD dual-hall calibration support
- manual hall sensor debug mode
- CAN + UART configuration interfaces

## Quick start

1. Build and flash with STM32CubeIDE.
2. Open a UART terminal and confirm the menu prints.
3. Enter setup mode (`s`) and verify key parameters (current limits, gains, CAN IDs).
4. Run needed calibration:
   - hall calibration: `h` (auto-selects HIP vs AB/AD from `MOTOR_POSITION`)
5. Enter motor mode (`m`) and begin command testing.

## Hardware + Toolchain

- MCU family: STM32F4 (project is configured for STM32F446)
- IDE: STM32CubeIDE
- Project file: `motorcontrol.ioc`
- Linker scripts:
  - `STM32F446RETX_FLASH.ld`
  - `STM32F446RETX_RAM.ld`

Hardware pin/peripheral mappings and low-level constants are centralized in:

- `Core/Inc/hw_config.h`

## Repository layout

- `Core/Src/main.c`: startup, config sanitization, peripheral bring-up, controller initialization
- `Core/Src/foc.c`: FOC + torque/position control loop
- `Core/Src/fsm.c`: runtime state machine and UART command handling
- `Core/Src/can.c`: CAN packet parsing/packing and register access protocol
- `Core/Src/position_sensor.c`: SPI encoder sampling and processing
- `Core/Src/calibration.c`: encoder phase ordering and calibration
- `Core/Src/abad_calibration.c`: AB/AD dual-hall calibration logic
- `Core/Inc/user_config.h` + `Core/Src/user_config.c`: persistent register map and validation

## Build and flash

### STM32CubeIDE

1. Open this folder in STM32CubeIDE.
2. Import as existing STM32 project if needed.
3. Build the `Debug` configuration.
4. Flash with ST-LINK from IDE run/debug controls.

### Makefile path

A generated makefile is present under `Debug/makefile` if you prefer CLI workflows after Cube generation.

## Runtime architecture

The control flow is interrupt-driven and state-machine controlled.

- Fast control path:
  - ADC sampling + encoder sampling
  - torque/position control (`torque_control`)
  - FOC voltage/current regulation
- State control path (`run_fsm`):
  - `MENU_MODE`
  - `MOTOR_MODE`
  - `SETUP_MODE`
  - `ENCODER_MODE`
  - `ENCODER_CALIBRATE`
  - `HALL_CALIBRATE`
  - `HALL_DEBUG_MODE`
  - `ABAD_CALIBRATE`

## Calibration control logic

This firmware has two hall-based calibration paths with different sensing layouts.

```text
Hip hall calibration (single sensor)

MENU_MODE --(h)--> HALL_CALIBRATE
HALL_CALIBRATE:
	read hall -> detect edges -> compute midpoint(+offset) -> align target
	| success: encoder_set_zero + state=SUCCESS -> MOTOR_MODE
	| fail: exceeded search window -> state=FAIL -> MENU_MODE
```

## Hip hall calibration (`HALL_CALIBRATE`)

Implemented in `fsm.c` (`hall_calibrate`) using a single binary hall input.

Control sequence:

1. Enter mode from FSM (`HALL_CALIBRATE`) and set calibration PID gains (`HALL_CAL_KP/KI/KD`).
2. Read hall input each control cycle.
3. Detect edges by comparing current and previous hall state.
4. Record two transition positions:
   - magnet entering sensor window (1 -> 0)
   - magnet leaving sensor window (0 -> 1)
5. Compute midpoint between those two mechanical positions.
6. Apply optional offset (`HALL_CAL_OFFSET`) and move slowly (`HALL_CAL_SPEED`) toward target.
7. When target reached:
   - set success code
   - call zeroing path (`encoder_set_zero`)
   - transition back to motor mode
8. If motion exceeds expected search range, fail and return to menu.

Key idea: the hip routine finds the center of one hall detection window and uses that as the zero reference (plus optional offset).

## AB/AD dual-hall calibration (`ABAD_CALIBRATE`)

Implemented in `abad_calibration.c` (`abad_hall_calibrate`) using two binary hall sensors (`HALL_A_IO`, `HALL_B_IO`).

Control sequence:

1. Enter Hall Calibration (`h`) from menu.
2. FSM routes automatically:
   - `MOTOR_POSITION == MOTOR_POS_HIP` -> `HALL_CALIBRATE`
   - otherwise -> `ABAD_CALIBRATE`
3. AB/AD calibration validates motor role (must not be HIP).
4. `FIND_BOTTOM_SENSOR`:
   - move in configured AB/AD direction
   - require expected bottom sensor detection within `ABAD_PROBE_TRAVEL_DEG`
5. `FIND_ZERO`:
   - continue moving until first `A && B` trigger (forward trigger sample)
6. `CENTER_SAMPLE_REVERSE`:
   - overtravel a few degrees
   - reverse direction
   - capture second `A && B` trigger (reverse trigger sample)
7. `CENTER_ZERO`:
   - compute midpoint between forward/reverse trigger samples
   - command target to that midpoint and settle
8. On success:
   - call `abad_encoder_set_zero`
   - transition to `MOTOR_MODE`

Key idea: AB/AD zero is centered from two-direction trigger samples of the same overlap window.

## Interfaces

### UART (setup + menu)

Terminal menu is driven by single-character commands and setup register writes.

High-level menu commands in `fsm.h`:

- `m`: enter motor mode
- `c`: encoder calibration mode
- `e`: encoder display mode
- `s`: setup mode
- `z`: set encoder zero
- `h`: hall calibration mode
- `q`: hall sensor debug mode (raw + active prints for HIP/A/B halls)
- `ESC` (`27`): return to menu from active mode

### UART parser behavior

The setup parser (`process_user_input` in `fsm.c`) uses this flow:

1. First typed character in setup mode becomes `cmd_id` (parameter prefix).
2. Remaining characters are buffered as ASCII value text.
3. On `ENTER` (`13`), the firmware tries:
   - float register update (`float_reg_update_uart`)
   - then int register update (`int_reg_update_uart`) if float command did not match
4. Firmware prints one response line back over UART.

Input format:

- `<prefix><value><ENTER>`
- Example: `b1000<ENTER>` sets current-loop bandwidth to 1000.

Range and validation behavior:

- Out-of-range values return `Not a valid value`.
- Unknown prefixes return `Not a valid command`.
- Some channels enforce coupled updates:
  - setting `V_MAX` also sets `V_MIN = -V_MAX`
  - setting `T_MAX` also sets `T_MIN = -T_MAX`

### Common setup prefixes

Most-used writable prefixes exposed in `enter_setup_state` and `user_config.h`:

- `g`: gear ratio (`GR`)
- `t`: torque constant (`KT`)
- `b`: current bandwidth (`I_BW`)
- `l`: current limit (`I_MAX`)
- `p`: max position setpoint (`P_MAX`)
- `v`: max velocity setpoint (`V_MAX`)
- `T`: max torque setpoint (`T_MAX`)
- `k`, `i`, `d`: max position-loop gains (`KP_MAX`, `KI_MAX`, `KD_MAX`)
- `X`, `Y`, `Z`: default motor-mode gains (`MOTOR_MODE_KP/KI/KD`)
- `f`: field-weakening current limit (`I_FW_MAX`)
- `c`: continuous current limit (`I_MAX_CONT`)
- `a`: calibration current (`I_CAL`)
- `P`: motor position role (`MOTOR_POSITION`)
- `r`: hall calibration direction (`HALL_CAL_DIR`)
- `e`: hall calibration offset (`HALL_CAL_OFFSET`)
- `h`: hall calibration speed (`HALL_CAL_SPEED`)
- `K`, `I`, `D`: hall calibration gains (`HALL_CAL_KP/KI/KD`)
- `R`: AB/AD calibration direction (`ABAD_CAL_DIR`)
- `O`: AB/AD calibration offset (`ABAD_CAL_OFFSET`)
- `S`: AB/AD calibration speed (`ABAD_CAL_SPEED`)
- `J`, `U`, `V`: AB/AD calibration gains (`ABAD_CAL_KP/KI/KD`)
- `n`: CAN node ID (`CAN_ID`)
- `m`: CAN master ID (`CAN_MASTER`)
- `o`: CAN timeout (`CAN_TIMEOUT`)

Note: any setup item shown in the table that has no UART command prefix in `user_config.h` is read-only over UART and must be changed via other configuration paths.

If any setup-menu printout and command table ever disagree, treat the `CMD_*` mappings in `Core/Inc/user_config.h` as source of truth.

### Complete writable UART prefix reference

The table below lists all register prefixes that are writable over UART (`CMD_* != ' '`).

| Prefix | Register          | Type  | Notes                                            |
| ------ | ----------------- | ----- | ------------------------------------------------ |
| `a`    | `I_CAL`           | float | Calibration current                              |
| `b`    | `I_BW`            | float | Current loop bandwidth                           |
| `c`    | `I_MAX_CONT`      | float | Continuous current limit                         |
| `d`    | `KD_MAX`          | float | Max velocity gain                                |
| `e`    | `HALL_CAL_OFFSET` | float | Hall calibration offset                          |
| `f`    | `I_FW_MAX`        | float | Field-weakening current limit                    |
| `g`    | `GR`              | float | Gear ratio                                       |
| `h`    | `HALL_CAL_SPEED`  | float | Hall calibration speed                           |
| `i`    | `KI_MAX`          | float | Max integral gain                                |
| `k`    | `KP_MAX`          | float | Max position gain                                |
| `l`    | `I_MAX`           | float | Current limit                                    |
| `n`    | `CAN_ID`          | int   | CAN node ID                                      |
| `m`    | `CAN_MASTER`      | int   | CAN master/controller ID                         |
| `o`    | `CAN_TIMEOUT`     | int   | CAN timeout cycles                               |
| `p`    | `P_MAX`           | float | Max position setpoint                            |
| `P`    | `MOTOR_POSITION`  | int   | Motor role (`0=HIP, 1=FL/RR, 2=FR/RL`)           |
| `r`    | `HALL_CAL_DIR`    | int   | Hall calibration direction (`-1` or `1`)         |
| `R`    | `ABAD_CAL_DIR`    | int   | AB/AD calibration direction (`-1` or `1`)        |
| `t`    | `KT`              | float | Torque constant                                  |
| `v`    | `V_MAX`           | float | Max velocity setpoint (also sets `V_MIN=-V_MAX`) |
| `T`    | `T_MAX`           | float | Max torque setpoint (also sets `T_MIN=-T_MAX`)   |
| `D`    | `HALL_CAL_KD`     | float | Hall calibration derivative gain                 |
| `I`    | `HALL_CAL_KI`     | float | Hall calibration integral gain                   |
| `J`    | `ABAD_CAL_KP`     | float | AB/AD calibration proportional gain              |
| `K`    | `HALL_CAL_KP`     | float | Hall calibration proportional gain               |
| `O`    | `ABAD_CAL_OFFSET` | float | AB/AD calibration offset                         |
| `S`    | `ABAD_CAL_SPEED`  | float | AB/AD calibration speed                          |
| `U`    | `ABAD_CAL_KI`     | float | AB/AD calibration integral gain                  |
| `V`    | `ABAD_CAL_KD`     | float | AB/AD calibration derivative gain                |
| `X`    | `MOTOR_MODE_KP`   | float | Default motor-mode proportional gain             |
| `Y`    | `MOTOR_MODE_KI`   | float | Default motor-mode integral gain                 |
| `Z`    | `MOTOR_MODE_KD`   | float | Default motor-mode derivative gain               |

Reserved/unused in UART setup parser:

- Prefix `'_'` appears in internal definitions (for some min-side limits) and is not intended for normal setup usage.

### Example UART session

```text
<boot>
 Commands:
	m - Motor Mode
	c - Calibrate Encoder
   h - Hall Calibration
   q - Hall Sensor Debug
	s - Setup
	e - Display Encoder
	z - Set Zero Position

> s
<prints configuration table>

> b1200
I_BW set to 1200.000000

> l80
Not a valid value

> l40
I_MAX set to 40.000000

> <ESC>
<returns to menu>

Calibration command note:

- Use `h` only. Calibration type is auto-selected from `MOTOR_POSITION`.
```

## CAN

CAN supports:

- control command packets (position + gains + feed-forward torque)
- config read/write packets for int/float register map
- status reply packets (position/velocity/torque + state/version metadata)

Protocol encoding/decoding lives in:

- `Core/Src/can.c`

## Persistent config model

Persistent registers are stored in flash via the preference writer and exposed as:

- float register bank (`__float_reg`)
- int register bank (`__int_reg`)

Key files:

- `Core/Inc/user_config.h`
- `Core/Src/user_config.c`
- `Core/Src/preference_writer.c`

On boot, `main.c` sanitizes register values to safe defaults if flash contains invalid/out-of-range data.

## AB/AD support

AB/AD support uses two digital hall sensors and a dedicated calibration flow.

Sensor pins (default):

- `HALL_A_IO`: `GPIOB, GPIO_PIN_14`
- `HALL_B_IO`: `GPIOB, GPIO_PIN_15`
- Hip hall (`HALL_IO`): `GPIOC, GPIO_PIN_6`

Motor role selection is runtime-configurable:

- `MOTOR_POS_HIP`
- `MOTOR_POS_ABAD_FL_RR`
- `MOTOR_POS_ABAD_FR_RL`

This allows orientation-dependent direction handling without forking firmware per leg.

### AB/AD limits and command handling

- Mechanical limits are defined in `Core/Inc/abad_calibration.h`.
- Calibration operates inside a safer travel window and fails if exceeded.
- Runtime AB/AD command handling rejects out-of-range commands with printed error output (rather than silently applying them).

Startup probe and reset behavior:

- On entering `ABAD_CALIBRATE`, the firmware calls `abad_cal_reset()` to clear any previous transition history and initialize the internal command/estimate state so calibration starts cleanly.
- **Active direction probe**: before collecting hall transitions, the firmware probes the configured calibration direction (`ABAD_CAL_DIR` with motor-role inversion) using a short fixed-angle step (`ABAD_PROBE_STEP_DEG`) over `ABAD_PROBE_CYCLES` cycles. If hall transitions are detected, that direction is locked for the rest of calibration. If not, it performs exactly one opposite-direction attempt. If neither attempt produces transitions, calibration aborts with an explicit probe failure message.
- This active probe **requires no assumptions about encoder accuracy** at startup and automatically selects the productive direction regardless of mechanical position or encoder reference quality.

### Hall debug mode

- Enter from menu with `q`.
- Prints raw hall values and derived active state (`active = raw == 0`) for:
  - HIP hall (`PC6`)
  - AB/AD Hall A (`PB14`)
  - AB/AD Hall B (`PB15`)
- Prints immediate edge transitions (`prev -> new`) when any hall input changes.
- Mode is non-driving for safe manual magnet/wiring bring-up. Press `ESC` to return to menu.

#### Hall debug bring-up checklist

1. Power on, open UART, and enter hall debug mode with `q`.
2. Verify idle values first:
   - Expected raw values are typically `1 1 1` (HIP, A, B) when no magnet is over a sensor.
   - Expected active values are `0 0 0` at idle because active is defined as `raw == 0`.
3. Move a magnet over each sensor one-by-one:
   - Raw value for that sensor should toggle `1 -> 0` near the detection zone.
   - Active value for that sensor should toggle `0 -> 1` at the same moment.
   - UART should print an edge line for each transition.
4. Sweep through the full mechanical range slowly and confirm repeated transitions:
   - AB/AD axis should show A/B edges near expected magnet locations.
   - If no edges appear in sweep, calibration will fail in probe phase.
5. Press `ESC` to return to menu and then run AB/AD calibration (`a`) once transitions are confirmed.

## Motor breakout pinout (motor driver)

The current firmware pinout mapping for motor-breakout digital hall lines is:

| Breakout pin | MCU pin | Firmware role                            |
| ------------ | ------- | ---------------------------------------- |
| D0           | PC6     | Hip hall (`HALL_IO`)                     |
| D1           | PB14    | AB/AD Hall A (`HALL_A_IO`)               |
| D2           | PB15    | AB/AD Hall B (`HALL_B_IO`)               |
| D3           | PC7     | Spare/available (not used by hall logic) |

## Safety and fault behavior

- CAN timeout can zero commands in motor/hall modes.
- DRV gate enable/disable is managed during state entry/exit.
- Register writes are range-validated through config update helpers.
- Invalid configuration commands return explicit status/error codes.

## Adapting to new hardware

For porting to another board, start here:

1. `Core/Inc/hw_config.h` (pins, ADC/SPI/CAN instances, constants)
2. CubeMX `.ioc` peripheral setup
3. `user_config.h` defaults/limits for motor and robot mechanics
4. calibration constants and FSM mode defaults

## Related links

- Original integrated motor controller hardware:
  - https://github.com/bgkatz/3phase_integrated
- STM32CubeIDE:
  - https://www.st.com/en/development-tools/stm32cubeide.html
- Legacy/original documentation:
  - https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit?usp=sharing
