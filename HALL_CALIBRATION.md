# Hall calibration

How the joint finds its encoder zero from hall sensors. There are two routines, both in
`Core/Src/hall_calibration.c`:

- **Hip** (`hall_calibrate()`): one hall sensor, one magnet.
- **AB/AD** (`abad_hall_calibrate()`): two hall sensors (A, B), four magnets.

`MOTOR_POSITION` picks the routine: 0 = hip, 1 = AB/AD.

Contents:
[Common](#common) ·
[Hip calibration](#hip-calibration) ·
[AB/AD calibration](#abad-calibration) ·
[Hall debug mode (`q`)](#hall-debug-mode-q) ·
[AB/AD bench test](#abad-bench-test) ·
[Registers](#registers) ·
[Changelog](#changelog)

## Common

### Starting a calibration

- UART: `h` from the menu.
- CAN: function code `FC_HALL_CAL` (3).

Both go through the same path (`HALL_CAL_CMD` in `fsm.c`):

1. `encoder_set_zero()`, so every angle in the log is relative to where the joint was
   when cal started.
2. Enter `HALL_CALIBRATE`, load the calibration gains (`HALL_CAL_KP/KI/KD` for hip,
   `ABAD_CAL_KP/KI/KD` for AB/AD), enable the motor.
3. Run the selected routine every control cycle (40 kHz).

On success the encoder is zeroed at the found position and the FSM switches to
`MOTOR_MODE`. On failure the joint holds where it is and the FSM returns to `MENU_MODE`.
Aborting a run (`ESC`, CAN `FC_RESET`) sets the status back to 0 (uncalibrated).

**Set zero** (`z`, CAN `FC_SET_ZERO`) makes the current position 0 without moving the
motor, and sets the status to 0: status 2 only means the current zero came from a hall
calibration. Starting a calibration zeroes the encoder itself, so no set zero is needed
first. (fpga_driver `main` still sends `SET_ZERO` before `HALL_CALIBRATE` at power-up, left
over from before calibration zeroed itself; `fix/hall_calibration` removed it in `c157a1b`.)

**When the zero is saved:** the zero is a register (`M_ZERO`). Calibration and set zero only
change it in RAM, but every save to flash writes all registers, so the next config save (a
UART setup entry, or a CAN config write that changes a value) stores the current zero too.
After a power cycle the motor then starts from that saved zero. Recalibrate after power-up
regardless.

CAN `FC_CONTROL_CMD` frames are ignored while the FSM is in `HALL_CALIBRATE`, so a host
streaming commands can't fight the calibration.

The CAN timeout (`CAN_TIMEOUT`) is paused during calibration: the routine drives its own
commands and has its own failure limits. The counter is held at 0 and starts when
calibration hands over to `MOTOR_MODE`. From then on the host has to send control frames
within `CAN_TIMEOUT` cycles as usual, or the gains are zeroed.

### Status codes

`hall_cal_state` is reported in every CAN reply and in the calibration status frames
(`can_send_cal_status()`); poll it with `FC_GET_STATE` (6):

| Code | Name | Meaning |
|---|---|---|
| 0 | `CODE_HALL_UNCALIBRATED` | No calibration since boot, the last run was aborted, or set zero was used since |
| 1 | `CODE_HALL_CALIBRATING` | Running |
| 2 | `CODE_HALL_CAL_SUCCESS` | Done, encoder zeroed |
| 3 | `CODE_HALL_CAL_FAIL` | Failed, back in the menu |

### Pins

| Breakout pin | MCU pin | Role |
|---|---|---|
| D0 | PC6 | Hip hall (`HALL_IO`) |
| D1 | PB14 | AB/AD hall A (`HALL_A_IO`) |
| D2 | PB15 | AB/AD hall B (`HALL_B_IO`) |
| D3 | PC7 | Spare, not used |

All hall inputs are active low: `active = (pin == 0)`.

## Hip calibration

One sensor and one magnet. Finds the center of the sensor's detection window.

0. If the sensor is already on at start (joint on the magnet), back out against
   `HALL_CAL_DIR` until the sensor has been off for 3° (`HIP_BACKOUT_MARGIN_DEG`), so the
   window is then approached from outside.
1. Sweep in `HALL_CAL_DIR` (±1) at `HALL_CAL_SPEED`.
2. Record where the sensor turns on (magnet enters) and where it turns off (magnet leaves).
   The midpoint is the center.
3. The target is the center plus `HALL_CAL_OFFSET` degrees in the sweep direction. Drive
   to it from whichever side the joint is on (back toward the center with no offset,
   onward with a large one).
4. Zero the encoder there.

Fails if the joint sweeps a full turn (360° output) without finding both edges, backs out
a full turn without leaving the window (sensor stuck on), or if `HALL_CAL_DIR` isn't 1 or
−1 (the UART setup accepts 0).

Log lines:

| Message | Meaning |
|---|---|
| `HIP Hall Cal: starting  dir=CW  speed=0.2 rad/s` | Start. `CW` means `HALL_CAL_DIR` = +1, `CCW` means −1; the physical direction isn't checked |
| `HIP Hall Cal: started on the magnet, backing out` | Sensor on at start (step 0) |
| `HIP Hall Cal: clear of the magnet at X deg, sweeping` | Back-out done |
| `HIP Hall Cal: sweeping  pos=X deg` | Every 0.5 s while searching |
| `HIP Hall Cal: sensor enter  pos=X deg` | Magnet entered the sensor window |
| `HIP Hall Cal: sensor exit  pos=X deg  center=C deg` | Magnet left; center computed |
| `HIP Hall Cal: centering to T deg` | Moving to the target (center + offset) |
| `HIP Hall Cal: SUCCESS  center=C deg (now zero)` | Done, encoder zeroed |
| `HIP Hall Cal: FAIL  swept >360 deg without finding sensor` | No sensor window found in a full turn |
| `HIP Hall Cal: FAIL  backed out >360 deg without leaving the sensor window` | Sensor stuck on |
| `HIP Hall Cal: FAIL  HALL_CAL_DIR must be 1 or -1` | `HALL_CAL_DIR` is 0 |

## AB/AD calibration

### Hardware

- **4 magnets** on the moving side, evenly spaced (estimated ~15° apart; confirm with a
  [`q` sweep](#mapping-the-abad-sensors), as the measured gaps don't obviously fit 15°).
- **2 hall sensors, A and B**, on the fixed side, spaced to match the two **outer** magnets.

At the centered position, A is over one outer magnet and B is over the other. The two
inner magnets sit between the sensors, unsensed. That is the only position where both
sensors see a magnet, so the **zone** (A and B both active) is unique. Calibration zeroes
the encoder at the center of the zone.

```
centered: both active (the zone)

  sensors (fixed):     A                         B
  magnets (moving):   [O]------o--------o-------[O]
                      outer   inner    inner    outer
```

Each sensor only sees the inner magnets on one side of the zone. Rotate one way and the
magnets slide so that A passes over the inner magnets while B sees nothing; rotate the
other way and B passes over them while A sees nothing. In motor angle:

- **A side** (`ABAD_A_SIDE_DIR = -1`): toward −angle, A pulses on the inner magnets.
- **B side** (`ABAD_B_SIDE_DIR = +1`): toward +angle, B pulses on the inner magnets.

Going from A to B along the magnet arc is therefore +angle. Whether +angle is clockwise
or counterclockwise isn't set in firmware; it depends on how the encoder chip reads the
rotor. To find out, turn the output by hand in `q` or `e` mode and watch `ENC`.

### Same on every leg

Front AB/AD motors have the shaft pointing forward, rear ones backward. The hall board is
on the motor housing and the magnets are on the output, so the whole layout turns with
the motor. Turning the motor around, or moving it to the other side of the robot, changes
which motor direction lifts the leg, but not where the inner magnets sit in motor angle.
Calibration only needs the zone, not which way is "up", so it runs the same on every AB/AD
leg, and `MOTOR_POSITION` = 1 for all of them.

On FL/RR legs the A side is **leg up**; on FR/RL legs it is **leg down**.

If a leg is ever built with the hall board rotated or A/B wired the other way round, the
sides swap and calibration fails with "second wrong-side reversal". The
[`q` sweep](#mapping-the-abad-sensors) catches this.

### What the sensors read, by position

Measured on FL/RR legs (CAN ID 1024 and 1027) from calibration logs. Angles are relative
to the start of that run. Fill in the blanks from a `q` sweep.

| Region (from B side to A side) | A | B | Measured (ID 1024) |
|---|---|---|---|
| B-side inner magnets | 0 | pulses | B window ended ~27° before the zone (ID 1027: ~25°) |
| Gap | 0 | 0 | ~27° |
| B-side edge sliver | **1** | 0 | ~0.5° moving into the zone, ~3° moving out (hysteresis) |
| **Zone** | 1 | 1 | **18.3° wide** (−30.8° to −49.1°), center −39.95° |
| A-side edge sliver | 0 | **1** | ~3° |
| A-side inner magnets | pulses | 0 | not yet logged |
| Hard stop | | | ~40° past the zone on the leg-up side |

The edge slivers exist because the two outer magnet windows don't line up exactly. They
make a single active sensor at start ambiguous:

- **A only**: either the B-side sliver (zone is next to us, toward the A side) or A on an
  inner magnet out on the A side (zone is the other way).
- **B only**: the same, mirrored.

### Sequence

All angles are relative to the start (the encoder is zeroed on entry). The ±85° soft
limits (`ABAD_LIMIT_MIN/MAX_DEG`) are relative to the start too. `abad_cal_reset()`
re-samples the encoder and clears all state first.

**1. `FIND_FIRST`: pick a direction** from the sensors on the first cycle:

| At start | Action |
|---|---|
| Both active | `BACKOUT`: move toward the B side until the zone clears, then sweep toward the A side |
| A only | Sweep toward the A side |
| B only | Sweep toward the B side |
| Neither | Probe toward the A side up to `ABAD_PROBE_TRAVEL_DEG` (30°); once any sensor appears, keep going that way. Fail if nothing within 30°. |

"A only → A side" is right in the sliver and wrong on an A-side inner magnet; step 2
catches the wrong case.

**2. `FIND_ZERO`: sweep to the zone, reverse once if needed.** Sweep until A and B are
both active: that position is the **forward trigger**.

Wrong-side rule: moving toward a side, if that side's sensor turns on and then off again
(staying off for `ABAD_WATCH_DEBOUNCE_DEG`, 1.5°) before the zone appears, we're out on
that side heading away from the zone. Reverse. Only one reversal is allowed; a second one
means the sensors don't match the layout and cal fails.

Inner-magnet edges passed on the way in are counted in `InnerTransitions` (diagnostic
only). Fails if the ±85° soft limit is reached.

**3. `CENTER_REV`: cross the zone, come back from the other side.**

1. Keep going until both-active drops: the **far edge**. Fail if the zone is wider than
   `ABAD_CENTER_ZONE_MAX_DEG` (60°).
2. Overtravel `ABAD_CENTER_OVERTRAVEL_DEG` (8°) past the far edge, so the return starts
   from outside the zone (clears the sliver and hysteresis).
3. Reverse and catch both-active switching on again: the **reverse trigger**. Fail if it
   doesn't appear within 30° of the far edge.

Both triggers are zone *entries*, from opposite sides. The sensor lag shifts each one inward
by about the same amount, so their midpoint is the true center.

**4. `CENTER_ZERO`: go to the midpoint and zero.** Slew the command to the midpoint, hold
it there, and wait until the joint has been within 1° of it, with both sensors active, for
50 ms. Then `encoder_set_zero()` and `MOTOR_MODE`. Fails after 10 s.

### Expected paths

| Start | Path |
|---|---|
| B side, on an inner magnet | toward B → B drops → reverse → zone |
| B side, nothing active | probe toward A → B pulses on the way → sliver → zone |
| B-side sliver (A only, just off center) | toward A → zone, no reversal |
| In the zone | back out toward B → toward A → zone |
| A-side sliver (B only) | toward B → zone |
| A side, on an inner magnet | toward A → A drops → reverse → zone |
| A side, nothing active | probe toward A → A appears → A drops → reverse → zone |

The last row moves up to 30° further out on the A side before reversing. On FL/RR that's
toward the leg-up hard stop (~40° past the zone), so don't start a cal far out on that side.

### Log lines

Status line every 0.2 s and on every phase change:

```
[ABAD CAL] Phase: FIND_ZERO | DeltaFromStart: -5.01 deg | EncFrame: -5.01 deg | A_active=0 B_active=0 | InnerTransitions=1
```

`DeltaFromStart` restarts at 0 on each phase change and on a wrong-side reversal;
`EncFrame` is the angle since cal started.

| Message | Meaning |
|---|---|
| `started on A only, sweeping toward A side` | Step 1, one sensor active at start |
| `A found while probing, continuing toward A side` | Step 1, nothing was active at start |
| `both sensors active, backing out toward B side before centering` | Started in the zone |
| `cleared both-active zone, sweeping toward A side for center` | Backout finished |
| `B inner-magnet transition 2` | Passed an inner-magnet edge on the way in |
| `A dropped before zone, reversing toward B side` | Wrong-side rule fired |
| `forward both-active trigger at X deg, crossing zone to far edge` | First zone entry |
| `far zone edge at X deg (zone ~W deg wide), overtraveling 8.00 deg` | Far edge; W should repeat run to run |
| `overtravel complete, reversing for second trigger` | Turning back |
| `reverse both-active trigger at X deg, center target T deg` | Second zone entry, midpoint computed |
| `AB/AD Calibration SUCCESS - centered at X deg (target T deg, err E deg)` | Done; E should be well under 1° |

Failures (cal stops, joint holds position, back to `MENU_MODE`):

| Message | Likely cause |
|---|---|
| `motor configured as HIP` | `MOTOR_POSITION` is 0 |
| `no hall sensor detected within 30 deg` | Started far out on the A side with nothing active, or a dead sensor/wiring |
| `could not exit both-active zone within 30 deg` | A sensor stuck active |
| `sensor pattern inconsistent (second wrong-side reversal)` | A and B sides swapped on this leg, or a flickering sensor |
| `reached mechanical limit before both sensors active` | Never saw the zone in ±85°: dead sensor |
| `far zone edge not found within expected width` | Zone > 60°: a sensor stuck active |
| `reverse trigger not found within expected travel` | Zone not re-entered on the way back |
| `reached mechanical limit while centering` | Zone too close to the ±85° soft limit |
| `center approach timeout` | Joint can't reach the midpoint (blocked, or gains too low) |

### Constants

Compile-time, in `hall_calibration.h` / `hall_calibration.c`:

| Name | Value |
|---|---|
| `ABAD_A_SIDE_DIR` / `ABAD_B_SIDE_DIR` | −1 / +1 |
| `ABAD_PROBE_TRAVEL_DEG` | 30° |
| `ABAD_LIMIT_MIN/MAX_DEG` | ±85° from start |
| `ABAD_WATCH_DEBOUNCE_DEG` | 1.5° |
| `ABAD_CENTER_OVERTRAVEL_DEG` | 8° |
| `ABAD_CENTER_ZONE_MAX_DEG` | 60° |
| `ABAD_CENTER_FINAL_TOL_DEG` | 1° |
| `ABAD_CENTER_SETTLE_CYCLES` | 2000 (50 ms) |
| `ABAD_CENTER_ZERO_TIMEOUT_CYCLES` | 400000 (10 s) |

### AB/AD limits outside calibration

The same ±85° limits also apply in normal operation: with `MOTOR_POSITION` = 1,
`torque_control()` (`foc.c`) refuses a position command outside them. It zeros the
current command and prints `Error: AB/AD command X deg outside mechanical limits`. The
limits are measured from the encoder zero, so they only mean anything after a calibration.

## Hall debug mode (`q`)

`q` from the menu. The motor is not driven, so the joint can be moved by hand. Prints the
state of all three hall inputs once on entry, then one line per edge with the encoder
angle:

```
Hall Debug init: PC6(HIP)=1 PB14(A)=1 PB15(B)=1 | active HIP=0 A=0 B=0 | ENC=0.00 deg
Hall edge: PC6(HIP) 1->1 | PB14(A) 1->0 | PB15(B) 1->1 | ENC=-30.31 deg
```

`ESC` returns to the menu.

### Bring-up check

1. Enter `q`. With no magnet over a sensor, raw values are `1 1 1` and active `0 0 0`.
2. Move a magnet over each sensor in turn: raw goes `1 -> 0` and an edge line prints.
3. Move the joint slowly through its full range and check the edges are where you expect.
   If no edges appear, calibration will fail.

### Mapping the AB/AD sensors

Move the leg slowly by hand from one hard stop to the other and back. Record every A and B
edge. Check:

1. A only switches on and off on the −angle side of the zone and B only on the +angle side.
   If it's the other way round, swap `ABAD_A_SIDE_DIR` / `ABAD_B_SIDE_DIR`.
2. Zone width and sliver widths in both directions (they differ by hysteresis).
3. Distance from the zone to each hard stop.
4. Which way is +angle: turn the output clockwise (looking from the motor body toward the
   shaft) and see whether `ENC` goes up.

Fill in the [sensor table](#what-the-sensors-read-by-position).

## AB/AD bench test

### Setup

- One AB/AD module on the bench, leg free to swing, hand on the supply output.
- UART terminal open (115200).
- Check registers in setup mode (`s`): `MOTOR_POSITION` = 1;
  `ABAD_CAL_SPEED` (default 0.25 rad/s); `ABAD_CAL_KP/KI/KD` (default 50 / 0 / 1.5).
- Trigger with UART `h`, or over CAN from the host. Run every case over UART first, then
  repeat the passing ones over CAN once.

### Step 0: map the sensors

On one FL/RR leg and one FR/RL leg, do the [`q` sweep](#mapping-the-abad-sensors) and save
the `Hall edge` lines. Pass if A only switches on the −angle side of the zone and B only on
the +angle side.

### Cases

Record for each run: start position, sensors at start, messages printed, forward/reverse
trigger, center target, final err, result code, time taken.

| # | Start position | Expected path | Pass if |
|---|---|---|---|
| 1 | B side, nothing active (between inner magnets) | `A/B found while probing, continuing toward A side` → zone → CENTER_REV → CENTER_ZERO | SUCCESS, err well under 1° |
| 2 | B side, on a B inner magnet (B only) | `started on B only, sweeping toward B side` → `B dropped before zone, reversing toward A side` → zone | SUCCESS, one reversal |
| 3 | Just off center on the B side (A only, sliver) | `started on A only, sweeping toward A side` → zone | SUCCESS, **no reversal, no jiggle** |
| 4 | Inside the zone, once on each side of center | `backing out toward B side` → `cleared both-active zone` → zone | SUCCESS |
| 5 | Just off center on the A side (B only, sliver) | `started on B only, sweeping toward B side` → zone | SUCCESS, no reversal |
| 6 | A side, on an A inner magnet (A only) | `started on A only` → `A dropped before zone, reversing toward B side` → zone | SUCCESS, one reversal |
| 7 | A side, nothing active, within ~25° of the zone | probe toward A → `A found while probing` → `A dropped …, reversing` → zone | SUCCESS — **on FL/RR this probes toward the leg-up stop, keep a hand on the supply** |
| 8 | Repeat case 1 five times, same start | — | Spread of final position (read encoder after cal) ≤ 0.5° |
| 9 | One start from each of cases 1–6 | — | Same physical zero each time (mark the leg); zone width repeats within 1–2° |
| 10 | FR/RL module, cases 1, 3, 4, 6 | Same messages as FL/RR; the leg moves the mirrored way | SUCCESS. `second wrong-side reversal` means A/B sides are swapped on this leg |
| 11 | Unplug sensor B, start on the B side | Zone never appears | FAIL `reached mechanical limit…` — **keep a hand on the supply** |
| 12 | During case 1, the host sends CONTROL frames (or `cansend` FC 5 with kp=500) | Frames ignored during `HALL_CALIBRATE` | Cal completes normally; no jump in p_des/kp |
| 13 | Hip module, `MOTOR_POSITION` = 0, `h` | Hip `hall_calibrate()` | Regression: still SUCCESS |

### Risks to watch

1. **Sweep range is relative, not mechanical.** The ±85° soft limits are measured from
   the start, so FIND_ZERO can drive up to 85° before failing. With a dead sensor the leg
   can reach the hard stop first and push into it at `ABAD_CAL_KP` until the soft limit
   trips.
2. **Probing with nothing active goes toward the A side** for up to 30°. On FL/RR that's
   toward the leg-up hard stop (~40° past the zone).
3. **Side assignment is assumed, not detected.** If a leg's A/B sides are swapped, cal
   fails on the second reversal. Step 0 catches it.
4. **The zero is only saved by the next config save** (see [Common](#common)). Confirm a
   power cycle without a config save loses it, and plan to re-cal after every power-up.

### Result template

```
date / board id / MOTOR_POSITION / fw commit:
case | start | sensors at start | messages | fwd trig | rev trig | target | err | code | time | notes
```

## Registers

Set over UART in setup mode (`s`, then `<prefix><value><ENTER>`) or over CAN
(`FC_MANAGE_CONFIG`). Defaults are applied at boot if flash holds an invalid value.

| Prefix | Register | Default | Range | Used by |
|---|---|---|---|---|
| `P` | `MOTOR_POSITION` | 0 | 0–1 | 0 = hip, 1 = AB/AD (any leg). A stored 2 (old FR/RL value) is converted to 1 at boot |
| `r` | `HALL_CAL_DIR` | 1 | −1 or 1 | Hip sweep direction |
| `h` | `HALL_CAL_SPEED` | 0.25 rad/s | 0–10 | Hip sweep speed |
| `e` | `HALL_CAL_OFFSET` | 0° | 0–143 | Hip: zero this far past the window center |
| `K`, `I`, `D` | `HALL_CAL_KP/KI/KD` | 50 / 0 / 1.5 | | Hip position gains during cal |
| `S` | `ABAD_CAL_SPEED` | 0.25 rad/s | 0–10 | AB/AD sweep speed |
| `J`, `U`, `V` | `ABAD_CAL_KP/KI/KD` | 50 / 0 / 1.5 | | AB/AD position gains during cal |

Two AB/AD registers were removed and their slots left empty so the flash layout doesn't
change: int register 7 (`ABAD_CAL_DIR`, `R`), because the AB/AD directions are fixed in
motor angle, and float register 36 (`ABAD_CAL_OFFSET`, `O`), which no calibration code ever
read. Setting `R` or `O` over UART returns "invalid command"; CAN writes to those addresses
return read-only.

## Changelog

### 2026-10-16

- **AB/AD directions in motor angle, same on every leg.** Start direction: A only → A
  side, B only → B side, both → back out toward B, none → probe toward A. While sweeping
  toward a side, if that side's sensor pulses on and off before the zone, reverse once; a
  second reversal fails. Fixes the jiggle on near-center starts and the runaway on starts
  above center (`e17052a`).
- **`MOTOR_POSITION` is now 0 = hip, 1 = AB/AD.** The FL/RR (1) vs FR/RL (2) split no longer
  changes anything; a stored 2 is converted to 1 at boot, and the setup-menu range is 0–1.
- **AB/AD zero accuracy.** `CENTER_ZERO` drives the command to the exact midpoint before
  counting settle; it used to zero up to 1° short on the approach side. Settle is 50 ms,
  timeout 10 s (`e17052a`).
- **AB/AD centering crosses the whole zone.** `CENTER_REV` runs to the far edge, overtravels
  8° and comes back, instead of a fixed 15° overtravel that ended inside zones wider than
  15° (`3a8878b`).
- **Removed `ABAD_CAL_DIR`** (`R`, `c5e8fc1`) and **`ABAD_CAL_OFFSET`** (`O`); neither was ever read.
- **CAN timeout paused during calibration.** It used to zero the gains mid-cal when no control
  frames arrived (UART-started cal, or a host that only polls), since control frames are
  ignored during cal. It now restarts when calibration ends.
- Status line field `BottomTransitions` renamed `InnerTransitions`.
- **Hip routine simplified and fixed.** The four direction/offset branches are now one.
  Fixes: a second calibration without a reboot could compute the center from the previous
  run's entry edge (the sensor reading wasn't reset between runs); an offset smaller than
  half the window zeroed at the exit edge instead of the target; `HALL_CAL_DIR` = 0 left the
  calibration stuck instead of failing.
- **Aborted calibration resets the status.** `ESC`/`FC_RESET` mid-run left `hall_cal_state`
  at 1 (calibrating), so a polling host kept seeing "calibrating". It's now 0. The hip
  routine also gets an explicit reset on entry (`hall_cal_reset()`, like
  `abad_cal_reset()`); before, a hip re-run after an abort wasn't recognised as a new run
  and failed.
- **Hip starts on the magnet backs out first.** Starting inside the sensor window used to take
  the start position as the entry edge (center off by up to half the window), and re-running
  a calibration with the leg still at its zero (on the magnet) failed with "swept >360 deg".

### 2026-07-16

- **AB/AD start direction from the sensors.** `FIND_BOTTOM_SENSOR` was replaced by
  `FIND_FIRST_SENSOR`, with a `BACKOUT` phase when starting inside the zone.
- **AB/AD zero search bounded by mechanical travel** instead of failing after 3 bottom-sensor
  transitions, which tripped when starting far below the zone.

### 2026-06-16

- **Hip and AB/AD calibration in one file.** `abad_calibration.{c,h}` became
  `hall_calibration.{c,h}`, and the hip `hall_calibrate()` moved out of `fsm.c` into it.
- **AB/AD calibration runs inside `HALL_CALIBRATE`**, selected by `MOTOR_POSITION`, with one
  entry point (`h` / `FC_HALL_CAL`) for both (`eb249ca`, `de077c9`).
- **One status and zeroing path.** `abad_cal_state` merged into `hall_cal_state`, and
  `abad_encoder_set_zero` into `encoder_set_zero` (`de077c9`, `70791fc`).

### 2026-06-15

- **CAN/UART parity.** Both entry paths run the same calibration routine (`e2071aa`).
- **Auto set-zero before calibration, plus status streaming** with `can_send_cal_status()`
  and more progress logging (`8bc5bad`).
