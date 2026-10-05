# AB/AD hall calibration — bench test

Tests `abad_hall_calibrate()` (`Core/Src/hall_calibration.c`) as of `c25430f`
(bidirectional, bottom-sensor safeguard removed), plus the control-frame guard
added to `stm32f4xx_it.c`.

## Setup

- One AB/AD module on the bench, leg free to swing, hand on the supply output.
- UART terminal open (115200) — every phase change prints `[ABAD CAL] Phase: ...`.
- Check registers from the setup menu before starting:
  - `MOTOR_POSITION` = 1 (normal: FL/RR) or 2 (mirrored: FR/RL). 0 is hip and
    runs the old single-sensor routine instead (case 12).
  - `ABAD_CAL_SPEED` (default 0.25 rad/s), `ABAD_CAL_KP/KI/KD` (default 50 / 0 / 1.5).
- Trigger: UART `h` from the menu, or over CAN via fpga_driver (Power Up → INIT).
  Run every case over UART first; repeat the pass cases over CAN once.

Angles below are joint angles relative to where the leg sits when cal starts
(`HALL_CAL_CMD` calls `encoder_set_zero()` first). "Zone" = both sensors active.

## Cases

Record for each run: start position, phases printed, `forward/reverse trigger`,
`center target`, final `err`, result code (2 = success, 3 = fail), time taken.

| # | Start position | Expected path | Pass if |
|---|---|---|---|
| 1 | Below the zone, bottom sensor inactive | FIND_FIRST → FIND_ZERO (up) → CENTER_REV → CENTER_ZERO | SUCCESS, err ≤ 1° |
| 2 | On the bottom sensor only | FIND_FIRST → FIND_ZERO (up) → … | SUCCESS |
| 3 | Inside the zone (both active) | FIND_FIRST → BACKOUT (down) → FIND_ZERO (up) → … | SUCCESS |
| 4 | On the top sensor only | FIND_FIRST → FIND_ZERO (**down**) → … | SUCCESS |
| 5 | Above the zone, nothing active | FIND_FIRST moves up ≤ 30° and finds nothing | FAIL `no hall sensor detected within 30 deg`, leg holds, MENU_MODE |
| 6 | Far below the zone (many magnets on the bottom sensor) | FIND_ZERO counts bottom transitions, keeps going | SUCCESS; note the transition count |
| 7 | Repeat case 1 five times, same start | — | Spread of final position (read encoder after cal) ≤ 1° |
| 8 | Repeat case 1 from 3 different starts | — | Same physical zero each time (mark the leg) |
| 9 | Mirrored module (`MOTOR_POSITION` = 2), cases 1, 3, 4 | Direction and bottom sensor swap | SUCCESS, moves the mirrored way |
| 10 | Unplug sensor B (top for normal), start below | FIND_ZERO never sees zone | FAIL `reached mechanical limit…` — **see risk 1, keep a hand on the supply** |
| 11 | During case 1, fpga_driver sends CONTROL frames (or `cansend` FC 5 with kp=500) | Frames ignored while `state == HALL_CALIBRATE` | Cal completes normally; no jump in p_des/kp |
| 12 | Hip module, `MOTOR_POSITION` = 0, `h` | Old single-sensor `hall_calibrate()` | Regression: still SUCCESS |

## Known risks to watch (from reading the code)

1. **Sweep range is relative, not mechanical.** The ±85° soft limits are measured
   from the start position, so FIND_ZERO can drive up to 85° before declaring
   failure. With a dead sensor or wrong `MOTOR_POSITION` the leg can reach the
   real hard stop first and push into it at `ABAD_CAL_KP` until the soft limit
   trips. Since `c25430f`, no bottom-sensor check stops it sooner.
2. **No timeout in CENTER_REV overtravel.** If the forward zone trigger happens
   within 15° of the +85° soft limit, the 15° overtravel can never complete
   (p_des is clamped) and the motor sits in HALL_CALIBRATE indefinitely. It is
   unlikely (needs a start ~70°+ from the zone) but worth one deliberate try
   during case 6. The host's cal step then never sees state 2.
3. **Zero is RAM-only.** `encoder_set_zero()` does not flush to flash; a power
   cycle needs a re-cal. That is expected, so just confirm it is.

## Result template

```
date / board id / MOTOR_POSITION / fw commit:
case | start | phases | fwd trig | rev trig | target | err | code | time | notes
```
