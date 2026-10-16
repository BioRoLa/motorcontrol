# Hall calibration

How the joint finds its encoder zero from hall sensors. There are two routines, both in
`Core/Src/hall_calibration.c`:

- **Hip** (`hall_calibrate()`): one hall sensor, one magnet.
- **AB/AD** (`abad_hall_calibrate()`): two hall sensors (A, B), four magnets.

`MOTOR_POSITION` picks the routine: 0 = hip, 1 = AB/AD front-left/rear-right (FL/RR), 2 = AB/AD front-right/rear-left (FR/RL).

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

### Which way is up: `MOTOR_POSITION` 1 vs 2

Front AB/AD motors have the shaft pointing forward, rear ones backward. The hall board is
on the motor housing and the magnets are on the output, so the whole layout turns with
the motor: in motor angle the A/B pattern is the same on every leg. What differs is which
motor direction lifts the leg. The sensors can't tell, so it comes from `MOTOR_POSITION`:

| Leg | `MOTOR_POSITION` | Leg up | Sensor that pulses below the zone | Upper sensor on the board |
|---|---|---|---|---|
| FL, RR | 1 | −angle (A side) | B | B |
| FR, RL | 2 | +angle (B side) | A | A |

Checked on RR: raising the leg makes `ENC` go down. The board labels were checked on all
four legs. The calibration only uses "up" to choose its first move; finding and centering
the zone works the same either way.

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
| Hard stop | | | ~40° past the zone on the leg-up side; leg-down side: robot stand |

The edge slivers exist because the two outer magnet windows don't line up exactly. They
make a single active sensor at start ambiguous: A alone can mean the sliver next to the
zone or an A inner magnet out on the A side, and the same for B. That's why the
calibration doesn't pick a direction from the sensors; it always moves up first and
reverses if it turns out to be above the zone.

### Sequence

All angles are relative to the start (the encoder is zeroed on entry). The ±85° soft
limits (`ABAD_LIMIT_MIN/MAX_DEG`) are relative to the start too. `abad_cal_reset()`
re-samples the encoder and clears all state first.

**1. `FIND_FIRST`: move up.** The first move is always up, because moving down from an
unknown start can drive the leg into the robot stand:

| At start | Action |
|---|---|
| Both active | `BACKOUT`: move up until the zone clears, then sweep down into it |
| A or B active | Sweep up |
| Neither | Probe up to `ABAD_PROBE_TRAVEL_DEG` (30°); once any sensor appears, keep going up. Fail if nothing within 30°. |

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

| Start | Path | Lowest point |
|---|---|---|
| Below the zone (any sensors) | up → zone → cross → 8° over the top → back down to the top edge → center | The start: never below it |
| In the zone | up until clear → down → zone → cross → 8° below the bottom edge → back up → center | 8° below the zone's bottom edge |
| Just above the zone (down-side sensor alone) | up → upper sensor pulses on an inner magnet → reverse down → zone | 8° below the zone's bottom edge |
| Above the zone (upper sensor on an inner magnet, or nothing) | up → upper sensor drops → reverse down → zone | 8° below the zone's bottom edge |

So downward travel only starts once the zone is known to be below, and never goes lower
than 8° under the zone (the leg's neutral range).

**Don't start with the leg raised above center.** A start above the zone keeps moving up
until the upper sensor passes an inner magnet. On the B side the first inner magnet ends
about 27–37° past the zone (from the FL/RR logs), close to the ~40° leg-up hard stop on an
FR/RL leg. Legs normally rest low, so this only matters if someone lifts a leg before
calibrating.

### Log lines

Status line every 0.2 s and on every phase change:

```
[ABAD CAL] Phase: FIND_ZERO | DeltaFromStart: -5.01 deg | EncFrame: -5.01 deg | A_active=0 B_active=0 | InnerTransitions=1
```

`DeltaFromStart` restarts at 0 on each phase change and on a wrong-side reversal;
`EncFrame` is the angle since cal started.

| Message | Meaning |
|---|---|
| `A active at start, sweeping up` | Step 1, one sensor active at start |
| `A found while probing, sweeping up` | Step 1, nothing was active at start |
| `both sensors active, backing out upward before centering` | Started in the zone |
| `cleared both-active zone, sweeping down for center` | Backout finished |
| `B inner-magnet transition 2` | Passed an inner-magnet edge on the way in |
| `A dropped before zone, reversing down` | Wrong-side rule fired: started above the zone |
| `forward both-active trigger at X deg, crossing zone to far edge` | First zone entry |
| `far zone edge at X deg (zone ~W deg wide), overtraveling 8.00 deg` | Far edge; W should repeat run to run |
| `overtravel complete, reversing for second trigger` | Turning back |
| `reverse both-active trigger at X deg, center target T deg` | Second zone entry, midpoint computed |
| `AB/AD Calibration SUCCESS - centered at X deg (target T deg, err E deg)` | Done; E should be well under 1° |

Failures (cal stops, joint holds position, back to `MENU_MODE`):

| Message | Likely cause |
|---|---|
| `motor configured as HIP` | `MOTOR_POSITION` is 0 |
| `no hall sensor detected within 30 deg` | Started far above the zone with nothing active, or a dead sensor/wiring |
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

The same ±85° limits also apply in normal operation: with `MOTOR_POSITION` = 1 or 2,
`torque_control()` (`foc.c`) refuses a position command outside them. It zeros the
current command and prints `Error: AB/AD command X deg outside mechanical limits`. The
limits are measured from the encoder zero, so they only mean anything after a calibration.

## Hall debug mode (`q`)

`q` from the menu. The motor is not driven, so the joint can be moved by hand. Prints the
state of all three hall inputs once on entry, then one line per edge with the encoder
angle:

```
Hall Debug (1 = magnet): HIP=0 A=0 B=1 | ENC=-12.40 deg
Hall edge (1 = magnet): HIP 0->0 | A 0->1 | B 1->1 | ENC=-30.31 deg
```

Values are 1 when the sensor sees a magnet (the pins themselves are active low). `ENC` is
read straight from the encoder, so it follows the joint while you move it by hand.

`ESC` returns to the menu.

### Bring-up check

1. Enter `q`. With no magnet over a sensor, all values are `0`.
2. Move a magnet over each sensor in turn: it goes `0->1` and an edge line prints.
3. Move the joint slowly through its full range and check the edges are where you expect.
   If no edges appear, calibration will fail.

### Mapping the AB/AD sensors

Move the leg slowly by hand from one hard stop to the other and back. Record every A and B
edge. Check:

1. A only switches on and off on the −angle side of the zone and B only on the +angle side.
   If it's the other way round, swap `ABAD_A_SIDE_DIR` / `ABAD_B_SIDE_DIR`.
2. Zone width and sliver widths in both directions (they differ by hysteresis).
3. Distance from the zone to each hard stop.
4. Which way is up: raise the leg by hand. `ENC` should go down on FL/RR (checked on RR)
   and up on FR/RL. If not, `MOTOR_POSITION` is wrong for this leg.

Fill in the [sensor table](#what-the-sensors-read-by-position).

## AB/AD bench test

### Setup

- One AB/AD module on the bench, leg free to swing, hand on the supply output.
- UART terminal open (115200).
- Check registers in setup mode (`s`): `MOTOR_POSITION` = 1 (FL/RR) or 2 (FR/RL);
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
| 1 | Resting low, nothing active | `… found while probing, sweeping up` → zone → CENTER_REV → CENTER_ZERO | SUCCESS, err well under 1°, **never moves below the start** |
| 2 | Resting low, on an inner magnet (lower sensor only) | `… active at start, sweeping up` → zone | SUCCESS, never moves below the start |
| 3 | Just below center (upper sensor alone, sliver) | `… active at start, sweeping up` → zone | SUCCESS, no reversal, no jiggle |
| 4 | Inside the zone, once on each side of center | `backing out upward` → `cleared both-active zone, sweeping down` → zone | SUCCESS; lowest point ~8° under the zone |
| 5 | Just above center (lower sensor alone, sliver) | up → `… dropped before zone, reversing down` → zone | SUCCESS, one reversal |
| 6 | Above the zone, upper sensor on an inner magnet | up → `… dropped before zone, reversing down` → zone | SUCCESS, one reversal — **moves up first, keep a hand on the supply** |
| 7 | Above the zone, nothing active | probe up → upper sensor appears → drops → reverse down | SUCCESS — **closest to the leg-up hard stop, keep a hand on the supply** |
| 8 | Repeat case 1 five times, same start | — | Spread of final position (read encoder after cal) ≤ 0.5° |
| 9 | One start from each of cases 1–6 | — | Same physical zero each time (mark the leg); zone width repeats within 1–2° |
| 10 | FR/RL module (`MOTOR_POSITION` = 2), cases 1, 3, 4, 6 | Same messages as FL/RR; the first move is up on this leg too | SUCCESS. `second wrong-side reversal` means A/B sides are swapped on this leg |
| 11 | Unplug the upper sensor, start resting low | Zone never appears | FAIL `reached mechanical limit…` — **keep a hand on the supply** |
| 12 | During case 1, the host sends CONTROL frames (or `cansend` FC 5 with kp=500) | Frames ignored during `HALL_CALIBRATE` | Cal completes normally; no jump in p_des/kp |
| 13 | Hip module, `MOTOR_POSITION` = 0, `h` | Hip `hall_calibrate()` | Regression: still SUCCESS |

### Risks to watch

1. **Sweep range is relative, not mechanical.** The ±85° soft limits are measured from
   the start, so FIND_ZERO can drive up to 85° before failing. With a dead sensor the leg
   can reach the hard stop first and push into it at `ABAD_CAL_KP` until the soft limit
   trips.
2. **Starting above the zone moves up first**, up to the first inner magnet past the zone
   (or 30° when probing with nothing active), toward the leg-up hard stop.
3. **Up direction comes from `MOTOR_POSITION`.** Set wrong, the first move goes down, toward
   the stand. Check with step 0 (raise the leg, watch `ENC`).
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
| `P` | `MOTOR_POSITION` | 0 | 0–2 | 0 = hip, 1 = AB/AD FL/RR, 2 = AB/AD FR/RL (sets which way is up) |
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

- **AB/AD always moves up first.** Moving down from an unknown start could drive the leg
  into the robot stand. Starts below the zone never go below the start; starts in or above
  the zone go at most 8° below the zone's bottom edge. `MOTOR_POSITION` is 0 = hip, 1 =
  FL/RR, 2 = FR/RL again: the sensors can't tell which way is up, so it has to be set.
  **FR/RL boards flashed with any build since 438260e had their 2 converted to 1; set `P`
  back to 2.**
- **Hall debug mode (`q`)**: `ENC` now follows the joint (it was frozen at the last value
  from motor/calibration mode), values print as 1 = magnet, and the start line prints on
  every entry, not just the first.
- **AB/AD directions in motor angle, same on every leg** (start-direction part superseded by
  "always moves up first" above). Start direction: A only → A
  side, B only → B side, both → back out toward B, none → probe toward A. While sweeping
  toward a side, if that side's sensor pulses on and off before the zone, reverse once; a
  second reversal fails. Fixes the jiggle on near-center starts and the runaway on starts
  above center (`e17052a`).
- (Briefly `MOTOR_POSITION` was reduced to 0/1 with a stored 2 converted to 1 at boot;
  reverted above.)
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
