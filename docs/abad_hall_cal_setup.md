# AB/AD hall calibration: how it is set up

How the AB/AD joint finds its zero: the magnet and sensor layout, what the sensors read
at each position, how `abad_hall_calibrate()` (`Core/Src/hall_calibration.c`) uses that, and
what every log line means. Bench procedure: [abad_hall_cal_test.md](abad_hall_cal_test.md).

## Hardware

- **4 magnets** on the moving side, evenly spaced (estimated ~15° apart; confirm with the
  `q` sweep below, as the measured gaps do not obviously fit 15°).
- **2 hall sensors, A (`PB14`) and B (`PB15`)**, on the fixed side. They are spaced to
  match the two **outer** magnets.
- Sensors are active low: `active = (pin == 0)`.

At the centered position, A is over one outer magnet and B is over the other. The two inner
magnets sit between the sensors, unsensed. That is the only position where both sensors
see a magnet, so the **zone** (A and B both active) is unique. Calibration zeroes the
encoder at the center of the zone.

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

### Same on every leg; `MOTOR_POSITION` 1 vs 2 doesn't matter here

Front AB/AD motors have the shaft pointing forward, rear ones backward. The hall board is
on the motor housing and the magnets are on the output, so the whole layout turns with
the motor. Turning the motor around, or moving it to the other side of the robot, changes
which motor direction lifts the leg (that's why FL/RR = `MOTOR_POSITION` 1 and FR/RL = 2),
but not where the inner magnets sit in motor angle. Calibration only needs the zone, not
which way is "up", so it runs the same on every AB/AD leg. `MOTOR_POSITION` only has to be
non-zero (0 = hip, which runs the single-sensor `hall_calibrate()` instead).

On FL/RR legs the A side is **leg up**; on FR/RL legs it is **leg down**.

If a leg is ever built with the hall board rotated or A/B wired the other way round, the
sides swap and calibration will hit the "second wrong-side reversal" failure. Check with
the `q` sweep.

## What the sensors read, by position

Measured on FL/RR legs (CAN ID 1024 and 1027) from calibration logs. Angles are motor-side
joint degrees relative to the start of that run. Fill in the blanks from a `q` sweep.

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
are why a single active sensor at start is ambiguous:

- **A only**: either the B-side sliver (zone is next to us, toward the A side) or A on an
  inner magnet out on the A side (zone is the other way).
- **B only**: the same, mirrored.

### Measuring with `q` (Hall Sensor Debug)

`q` from the menu doesn't drive the motor. It prints one line per edge with the encoder
angle:

```
Hall edge: PC6(HIP) 1->1 | PB14(A) 1->0 | PB15(B) 1->1 | ENC=-30.31 deg
```

Move the leg slowly by hand from one hard stop to the other and back. Record every A and B
edge. Check:

1. A only switches on and off on the −angle side of the zone and B only on the +angle side.
   If it's the other way round, swap `ABAD_A_SIDE_DIR` / `ABAD_B_SIDE_DIR`.
2. Zone width and sliver widths in both directions (they differ by hysteresis).
3. Distance from the zone to each hard stop.

Fill in the table above.

## Calibration sequence

Entered with `h` from the menu or CAN `FC_HALL_CAL`. Both paths call `encoder_set_zero()`
first, so all angles in the log are relative to where the leg was when cal started. The
±85° soft limits (`ABAD_LIMIT_*_DEG`) are relative to that start too.
`abad_cal_reset()` re-samples the encoder and clears all state.

### 1. `FIND_FIRST`: pick a direction

Decided from the sensors on the first cycle:

| At start | Action |
|---|---|
| Both active | `BACKOUT`: move toward the B side until the zone clears, then sweep toward the A side |
| A only | Sweep toward the A side |
| B only | Sweep toward the B side |
| Neither | Probe toward the A side up to `ABAD_PROBE_TRAVEL_DEG` (30°); once any sensor appears, keep going that way. Fail if nothing within 30°. |

"A only → A side" is right when we're in the sliver and wrong when we're on an A-side inner
magnet; step 2 catches the wrong case.

### 2. `FIND_ZERO`: sweep to the zone, reverse once if needed

Sweep until A and B are both active. That position is the **forward trigger**.

**Wrong-side rule:** moving toward a side, if that side's sensor turns on and then off
again (staying off for `ABAD_WATCH_DEBOUNCE_DEG`, 1.5°) before the zone appears, we're out
on that side heading away from the zone. Reverse. Only one reversal is allowed; a second one
means the sensors don't match the layout and cal fails.

Inner-magnet edges passed on the way in are counted in `InnerTransitions` (diagnostic
only). Fails if the ±85° soft limit is reached.

### 3. `CENTER_REV`: cross the zone, come back from the other side

1. Keep going until both-active drops: the **far edge**. Fail if the zone is wider than
   `ABAD_CENTER_ZONE_MAX_DEG` (60°).
2. Overtravel `ABAD_CENTER_OVERTRAVEL_DEG` (8°) past the far edge so the return starts
   from outside the zone (clears the sliver and hysteresis).
3. Reverse and catch both-active switching on again: the **reverse trigger**. Fail if it
   doesn't appear within 30° of the far edge.

Both triggers are zone *entries*, from opposite sides. The sensor lag shifts each one inward
by about the same amount, so their midpoint is the true center.

### 4. `CENTER_ZERO`: go to the midpoint and zero

Slew the command to the midpoint, hold it there, and wait until the leg has been within
1° of it, with both sensors active, for 2000 cycles (50 ms). Then `encoder_set_zero()` and
switch to `MOTOR_MODE`. Fails after 10 s.

The zero lives in RAM only. A power cycle needs a new calibration.

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

## Log reference

Status line every 8000 cycles (0.2 s) and on every phase change:

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

Failures (cal stops, leg holds position, back to `MENU_MODE`):

| Message | Likely cause |
|---|---|
| `motor configured as HIP` | `MOTOR_POSITION` is 0 |
| `no hall sensor detected within 30 deg` | Started far out on the A side with nothing active, or a dead sensor/wiring |
| `could not exit both-active zone within 30 deg` | A sensor stuck active |
| `sensor pattern inconsistent (second wrong-side reversal)` | A and B sides swapped on this leg (see above), or a flickering sensor |
| `reached mechanical limit before both sensors active` | Never saw the zone in ±85°: dead sensor |
| `far zone edge not found within expected width` | Zone > 60°: a sensor stuck active |
| `reverse trigger not found within expected travel` | Zone not re-entered on the way back |
| `reached mechanical limit while centering` | Zone too close to the ±85° soft limit |
| `center approach timeout` | Leg can't reach the midpoint (blocked, or gains too low) |

## Constants

| Name | Value | Where |
|---|---|---|
| `ABAD_A_SIDE_DIR` / `ABAD_B_SIDE_DIR` | −1 / +1 | `hall_calibration.h` |
| `ABAD_PROBE_TRAVEL_DEG` | 30° | `hall_calibration.h` |
| `ABAD_LIMIT_MIN/MAX_DEG` | ±85° from start | `hall_calibration.h` |
| `ABAD_WATCH_DEBOUNCE_DEG` | 1.5° | `hall_calibration.c` |
| `ABAD_CENTER_OVERTRAVEL_DEG` | 8° | `hall_calibration.c` |
| `ABAD_CENTER_ZONE_MAX_DEG` | 60° | `hall_calibration.c` |
| `ABAD_CENTER_FINAL_TOL_DEG` | 1° | `hall_calibration.c` |
| `ABAD_CENTER_SETTLE_CYCLES` | 2000 (50 ms) | `hall_calibration.c` |
| `ABAD_CENTER_ZERO_TIMEOUT_CYCLES` | 400000 (10 s) | `hall_calibration.c` |
| `ABAD_CAL_SPEED` | 0.25 rad/s default | setup register `S` |
| `ABAD_CAL_KP/KI/KD` | 50 / 0 / 1.5 default | setup registers |
