# AB/AD hall calibration — bench test

Tests `abad_hall_calibrate()` (`Core/Src/hall_calibration.c`) with the motor-angle
(universal) direction rules, plus the control-frame guard added to `stm32f4xx_it.c`.
How the calibration works, the sensor layout and every log line:
[abad_hall_cal_setup.md](abad_hall_cal_setup.md).

## Setup

- One AB/AD module on the bench, leg free to swing, hand on the supply output.
- UART terminal open (115200) — every phase change prints `[ABAD CAL] Phase: ...`.
- Check registers from the setup menu before starting:
  - `MOTOR_POSITION` = 1 (FL/RR) or 2 (FR/RL); both run the same AB/AD routine.
    0 is hip and runs the single-sensor routine instead (case 13).
  - `ABAD_CAL_SPEED` (default 0.25 rad/s), `ABAD_CAL_KP/KI/KD` (default 50 / 0 / 1.5).
- Trigger: UART `h` from the menu, or over CAN via fpga_driver (Power Up → INIT).
  Run every case over UART first; repeat the pass cases over CAN once.

Angles are relative to where the leg sits when cal starts (both entry paths call
`encoder_set_zero()` first). "Zone" = both sensors active. "A side" = −angle, where A
pulses on the inner magnets; "B side" = +angle, where B does. On FL/RR the A side is leg up.

## Step 0: map the sensors with `q`

Before the cal cases, on each leg type (one FL/RR, one FR/RL): enter `q`, move the leg
slowly by hand from hard stop to hard stop and back, and save the `Hall edge` lines.
Pass if A only switches on/off on the −angle side of the zone and B only on the +angle
side. Fill in the region table in the setup doc (zone width, sliver widths, inner-magnet
positions, distance to each hard stop).

## Cases

Record for each run: start position, sensors at start, messages printed, `forward/reverse
trigger`, `center target`, final `err`, result code (2 = success, 3 = fail), time taken.

| # | Start position | Expected path | Pass if |
|---|---|---|---|
| 1 | B side, nothing active (between inner magnets) | `A/B found while probing, continuing toward A side` → zone → CENTER_REV → CENTER_ZERO | SUCCESS, err well under 1° |
| 2 | B side, on a B inner magnet (B only) | `started on B only, sweeping toward B side` → `B dropped before zone, reversing toward A side` → zone | SUCCESS, one reversal |
| 3 | Just off center on the B side (A only, sliver) | `started on A only, sweeping toward A side` → zone | SUCCESS, **no reversal, no jiggle** |
| 4 | Inside the zone, both halves (start once on each side of center) | `backing out toward B side` → `cleared both-active zone` → zone | SUCCESS |
| 5 | Just off center on the A side (B only, sliver) | `started on B only, sweeping toward B side` → zone | SUCCESS, no reversal |
| 6 | A side, on an A inner magnet (A only) | `started on A only` → `A dropped before zone, reversing toward B side` → zone | SUCCESS, one reversal |
| 7 | A side, nothing active, within ~25° of the zone | probe toward A → `A found while probing` → `A dropped …, reversing` → zone | SUCCESS — **on FL/RR this probes toward the leg-up stop, keep a hand on the supply** |
| 8 | Repeat case 1 five times, same start | — | Spread of final position (read encoder after cal) ≤ 0.5° |
| 9 | One start from each of cases 1–6 | — | Same physical zero each time (mark the leg); printed zone width repeats within 1–2° |
| 10 | FR/RL module (`MOTOR_POSITION` = 2), cases 1, 3, 4, 6 | Same messages as FL/RR; the leg physically moves the mirrored way | SUCCESS. A `second wrong-side reversal` failure here means A/B sides are swapped on this leg |
| 11 | Unplug sensor B, start on the B side | Zone never appears | FAIL `reached mechanical limit…` — **see risk 1, keep a hand on the supply** |
| 12 | During case 1, fpga_driver sends CONTROL frames (or `cansend` FC 5 with kp=500) | Frames ignored while `state == HALL_CALIBRATE` | Cal completes normally; no jump in p_des/kp |
| 13 | Hip module, `MOTOR_POSITION` = 0, `h` | Single-sensor `hall_calibrate()` | Regression: still SUCCESS |

## Known risks to watch (from reading the code)

1. **Sweep range is relative, not mechanical.** The ±85° soft limits are measured
   from the start position, so FIND_ZERO can drive up to 85° before declaring
   failure. With a dead sensor the leg can reach the real hard stop first and push
   into it at `ABAD_CAL_KP` until the soft limit trips.
2. **Probing with nothing active goes toward the A side** for up to 30°. On FL/RR
   that's toward the leg-up hard stop (~40° past the zone). Starting far out on that
   side can reach the stop before a sensor appears.
3. **Side assignment is assumed, not detected.** If a leg's A/B sides are swapped,
   cal fails on the second reversal rather than finding the zone. Step 0 catches it.
4. **Zero is RAM-only.** `encoder_set_zero()` does not flush to flash; a power
   cycle needs a re-cal. That is expected, so just confirm it is.

## Result template

```
date / board id / MOTOR_POSITION / fw commit:
case | start | sensors at start | messages | fwd trig | rev trig | target | err | code | time | notes
```
