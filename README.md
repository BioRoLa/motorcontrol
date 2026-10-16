# motorcontrol firmware

STM32F4 firmware for high-bandwidth 3-phase motor control, based on the integrated controller hardware used in the MIT Mini Cheetah ecosystem and related designs.

This repo targets STM32CubeIDE-generated projects and is designed for practical robot actuator control with:

- FOC current loop
- position/torque control
- encoder calibration
- hall-sensor zero calibration for hip (one sensor) and AB/AD (two sensors) joints, plus a hall debug mode: see [HALL_CALIBRATION.md](HALL_CALIBRATION.md)
- CAN + UART configuration interfaces

## Quick start

1. Build and flash with STM32CubeIDE.
2. Open a UART terminal and confirm the menu prints.
3. Enter setup mode (`s`) and verify key parameters (current limits, gains, CAN IDs).
4. Set `MOTOR_POSITION` (`P`: 0 = hip, 1 = AB/AD FL/RR, 2 = AB/AD FR/RL) and run hall calibration with `h` ([HALL_CALIBRATION.md](HALL_CALIBRATION.md)).
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
- `Core/Src/hall_calibration.c`: hall calibration for hip and AB/AD joints ([HALL_CALIBRATION.md](HALL_CALIBRATION.md))
- `Core/Inc/user_config.h` + `Core/Src/user_config.c`: persistent register map and validation
- `HALL_CALIBRATION.md`: hall calibration, hall debug mode, AB/AD bench test

## Build and flash

### STM32CubeIDE

1. Open this folder in STM32CubeIDE.
2. Import as existing STM32 project if needed.
3. Build the `Debug` configuration.
4. Flash with ST-LINK from IDE run/debug controls.

### Makefile path

STM32CubeIDE writes a makefile to `Debug/makefile` when it builds the project (the `Debug/` folder is not in git). After one IDE build you can rebuild from the command line with `make -C Debug`.

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

## Hall calibration

Hip and AB/AD hall calibration, the hall debug mode, the AB/AD bench test and the related registers are documented in [HALL_CALIBRATION.md](HALL_CALIBRATION.md).

## Interfaces

### UART (setup + menu)

Terminal menu is driven by single-character commands and setup register writes.

High-level menu commands in `fsm.h`:

- `m`: enter motor mode
- `c`: encoder calibration mode
- `e`: encoder display mode
- `s`: setup mode
- `z`: set encoder zero (the current position becomes 0; the motor doesn't move). Resets the hall calibration status to 0
- `h`: hall calibration ([HALL_CALIBRATION.md](HALL_CALIBRATION.md))
- `q`: hall sensor debug mode ([HALL_CALIBRATION.md](HALL_CALIBRATION.md#hall-debug-mode-q))
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
- UART setup and CAN `FC_MANAGE_CONFIG` writes go through the same checks (`float_reg_set` / `int_reg_set` in `user_config.c`), so they accept and reject the same values.
- `P_MIN/P_MAX`, `V_MIN/V_MAX`, `T_MIN/T_MAX` are read-only. They aren't motion limits: they're the CAN number scaling (position commands over 0..2π, velocity over ±45 rad/s, torque over ±20 N·m, both in replies and in the feed-forward torque command) and must equal the host's constants in `fpga_driver/include/can_packet.h`. They're set to those values at boot (`CAN_*` in `user_config.h`).

### Common setup prefixes

Most-used writable prefixes exposed in `enter_setup_state` and `user_config.h`:

- `g`: gear ratio (`GR`)
- `t`: torque constant (`KT`)
- `b`: current bandwidth (`I_BW`)
- `l`: current limit (`I_MAX`)
- `k`, `i`, `d`: max position-loop gains (`KP_MAX` ≤ 500, `KI_MAX` ≤ 10, `KD_MAX` ≤ 5, matching the host's CAN packing ranges)
- `X`, `Y`, `Z`: default motor-mode gains (`MOTOR_MODE_KP/KI/KD`)
- `f`: field-weakening current limit (`I_FW_MAX`)
- `c`: continuous current limit (`I_MAX_CONT`)
- `a`: calibration current (`I_CAL`)
- `P`: motor role (`MOTOR_POSITION`: 0 = hip, 1 = AB/AD FL/RR, 2 = AB/AD FR/RL)
- hall calibration registers (`r`, `e`, `h`, `K`, `I`, `D`, `S`, `J`, `U`, `V`): see [HALL_CALIBRATION.md](HALL_CALIBRATION.md#registers)
- `n`: CAN node ID (`CAN_ID`)
- `m`: CAN master ID (`CAN_MASTER`)
- `o`: CAN timeout (`CAN_TIMEOUT`)

Note: a register with no setup prefix (`CMD_* == ' '` in `user_config.h`) is read-only on both UART and CAN.

If any setup-menu printout and command table ever disagree, treat the `CMD_*` mappings in `Core/Inc/user_config.h` as source of truth.

### Complete writable UART prefix reference

The table below lists all register prefixes that are writable over UART (`CMD_* != ' '`).

| Prefix | Register          | Type  | Notes                                                |
| ------ | ----------------- | ----- | ---------------------------------------------------- |
| `a`    | `I_CAL`           | float | Calibration current                                  |
| `b`    | `I_BW`            | float | Current loop bandwidth                               |
| `c`    | `I_MAX_CONT`      | float | Continuous current limit                             |
| `d`    | `KD_MAX`          | float | Max velocity gain (≤ 5)                              |
| `f`    | `I_FW_MAX`        | float | Field-weakening current limit                        |
| `g`    | `GR`              | float | Gear ratio                                           |
| `i`    | `KI_MAX`          | float | Max integral gain (≤ 10)                             |
| `k`    | `KP_MAX`          | float | Max position gain (≤ 500)                            |
| `l`    | `I_MAX`           | float | Current limit                                        |
| `n`    | `CAN_ID`          | int   | CAN node ID                                          |
| `m`    | `CAN_MASTER`      | int   | CAN master/controller ID                             |
| `o`    | `CAN_TIMEOUT`     | int   | CAN timeout cycles                                   |
| `P`    | `MOTOR_POSITION`  | int   | Motor role (`0=HIP, 1=AB/AD FL/RR, 2=AB/AD FR/RL`)   |
| `t`    | `KT`              | float | Torque constant                                      |
| `X`    | `MOTOR_MODE_KP`   | float | Default motor-mode proportional gain                 |
| `Y`    | `MOTOR_MODE_KI`   | float | Default motor-mode integral gain                     |
| `Z`    | `MOTOR_MODE_KD`   | float | Default motor-mode derivative gain                   |

Hall calibration registers (`r`, `e`, `h`, `K`, `I`, `D`, `S`, `J`, `U`, `V`) are listed in [HALL_CALIBRATION.md](HALL_CALIBRATION.md#registers).

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

```

Hall calibration (`h`) picks hip or AB/AD from `MOTOR_POSITION`; see [HALL_CALIBRATION.md](HALL_CALIBRATION.md).

## CAN

CAN supports:

- control command packets (position + gains + feed-forward torque); ignored during hall calibration
- config read/write packets for int/float register map (`FC_MANAGE_CONFIG`, 1): same validation as UART setup; rejected in motor mode; saved to flash when the value changes
- status reply packets (position/velocity/torque + state/version metadata)
- read-only state poll `FC_GET_STATE` (6): replies with FSM state and `hall_cal_state` without touching the FSM
- hall calibration start `FC_HALL_CAL` (3): see [HALL_CALIBRATION.md](HALL_CALIBRATION.md)

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

## Safety and fault behavior

- CAN timeout (`CAN_TIMEOUT` > 0) zeroes commands in motor mode if no control frame arrives in time. It is paused during hall calibration and starts counting when calibration ends.
- DRV gate enable/disable is managed during state entry/exit.
- Register writes are range-validated through config update helpers.
- Invalid configuration commands return explicit status/error codes.
- Position-loop integrator is clamped so `ki * error` stays within the commandable torque, and reset while `ki == 0`.
- AB/AD joints (`MOTOR_POSITION` 1 or 2) refuse position commands outside ±85° of the calibrated zero ([HALL_CALIBRATION.md](HALL_CALIBRATION.md#abad-limits-outside-calibration)).

## Adapting to new hardware

For porting to another board, start here:

1. `Core/Inc/hw_config.h` (pins, ADC/SPI/CAN instances, constants)
2. CubeMX `.ioc` peripheral setup
3. `user_config.h` defaults/limits for motor and robot mechanics
4. calibration constants and FSM mode defaults

## Changelog

Hall calibration changes are logged in [HALL_CALIBRATION.md](HALL_CALIBRATION.md#changelog).

### 2026-10-16

- **UART/CAN register-write parity.** Both paths now share one set of checks. Previously a CAN write to `V_MAX`/`T_MAX` didn't update `V_MIN`/`T_MIN`, UART accepted `HALL_CAL_DIR = 0` (CAN rejected it), and UART could write registers that have no setup prefix by typing a space.
- **CAN scaling registers read-only.** `P/V/T_MIN` and `P/V/T_MAX` are fixed at boot to the host's packing ranges (0..2π, ±45, ±20). They used to be writable (`p`, `v`, `T` over UART, all six over CAN), and any change silently rescaled every command and reply.
- **Feed-forward torque decoded over `T_MIN..T_MAX`**, matching the host. It used to be decoded over ±`I_MAX·KT·GR` (±19.2 N·m with the defaults) while the host packs ±20, so every feed-forward torque arrived ~4% low, more on boards with other `I_MAX`/`KT`/`GR`.
- **Set zero resets the hall calibration status to 0**, so status 2 always means the current zero came from a hall calibration. A set zero after a calibration used to keep reporting "calibrated" with an arbitrary zero.

### 2026-10-05

- **Gain limits bounded.** `KP_MAX`/`KI_MAX`/`KD_MAX` are limited to 500 / 10 / 5, the host's CAN packing ranges, so a mis-set register can't silently rescale every gain (`8b8ce00`).
- **Integrator anti-windup.** The position-loop integrator no longer grows without bound while `ki == 0` and is clamped to the commandable torque (`8b8ce00`).
- **Control frames ignored during hall calibration**, so a stray `FC_CONTROL_CMD` can't overwrite the calibration's `p_des`/`kp` (`8b8ce00`).

### 2026-06-16

- **New read-only CAN function code `FC_GET_STATE` (6).** Polls FSM `state` and `hall_cal_state` via `pack_reply_default` without calling `update_fsm` or any `unpack_*`, so a status poll can never mutate the FSM from any mode (`68ebea8`).

### 2026-06-15

- **Initial-velocity fix.** Moved the `angle_multiturn` correction into `encoder_set_zero`, and cleared additional stale velocity data at reset so a spurious initial velocity is not reported after calibration/reset (`5d8a4ab`, `88ec20a`).
- **CAN init.** Auto retransmission enabled and receive FIFO lock disabled (`1b5b0ee`).

## Related links

- Original integrated motor controller hardware:
  - https://github.com/bgkatz/3phase_integrated
- STM32CubeIDE:
  - https://www.st.com/en/development-tools/stm32cubeide.html
- Legacy/original documentation:
  - https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit?usp=sharing
