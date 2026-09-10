# Jukti Titans — LFR Tuning Cheat Sheet (competition tomorrow)

This is the order to do things in tonight. Don't skip ahead to PID tuning
before the checks in Step 1 — a mechanical or wiring problem will look
exactly like a "bad tuning" problem and you'll waste hours chasing it.

## Step 1 — 15-minute sanity checks (do these first)

1. **Sensor polarity.** Power on (or reset) the robot — for about 2 seconds
   the OLED shows "Hold LEFT+RIGHT now". Press and hold both buttons
   together during that window (a short hold, ~0.3s, is enough) to enter
   SENSOR TEST mode, which shows live 1/0 for all 5 sensors. Slide the
   robot over the black line by hand — sensors over black should read `1`.
   If they read `1` over white instead, open the code and flip
   `SENSOR_ACTIVE_HIGH` from `true` to `false` near the top, re-upload.
   (If you miss the 2-second window, just press reset and try again.)

2. **Sensor height & level.** The HW-871 board should sit roughly
   5–8mm above the track, parallel to the ground, with all 5 sensors in
   a straight line perpendicular to the direction of travel. If one end
   is higher than the other (crooked mounting), that sensor will behave
   differently from the rest — reseat/re-glue it now, not after your
   first failed run.

3. **Per-channel threshold trimmers.** The HW-871 has a small potentiometer
   per channel. With the robot sitting half-on/half-off the black line,
   turn each channel's trimmer until that channel's onboard LED (if
   present) or the SENSOR TEST reading flips cleanly right at the edge of
   the line — not too sensitive (triggers on the white surface too), not
   too dead (doesn't trigger on black). Do this on the actual competition
   track surface/lighting if you can — ambient light changes the
   threshold you need.

4. **Motor direction.** Put the robot on the ground, enter the tuning
   menu, hold MID to start it on a straight black line. If it immediately
   spins in place or drives backward off the line instead of following
   it, either swap `LEFT_MOTOR_IS_A` (true/false) at the top of the code,
   or your L298N wiring is mismatched — try the code flip first, it's
   faster than re-wiring at midnight.

5. **Power.** Confirm the L298N is getting a battery voltage clearly
   above what your N20 motors need (check your battery pack rating) and
   that the Arduino's own 5V is stable — a sagging battery makes a robot
   that tuned perfectly on the bench suddenly wobble/stall on the actual
   track. Use a freshly charged pack for tuning, not one you've been
   running all evening.

## Step 2 — Use the on-robot menu

No re-flashing needed to change numbers. On boot:

```
Base:90    Kp:0.30
Kd:0.30    Srch:70
```

- **LEFT / RIGHT** = decrease/increase the highlighted value
- **Short-press MID** = move to the next value (Base → Kp → Kd → Srch)
- **Hold MID ~0.7s** = save + start the run
- **Hold MID again while running** = stop, display comes back, back to menu

Settings are saved to EEPROM every time you start a run, so they survive
power-off. Place the robot at START, adjust one thing, hold MID, watch,
repeat. This is the whole tuning loop — you'll do it 15–30 times tonight.

Note: while the robot is RUNNING, the display is deliberately frozen (no
sensor readout, no live numbers) — that's on purpose, so the Arduino spends
100% of its time on reading sensors and driving motors instead of talking to
the OLED. You'll only see live info again once you stop it.

## Step 3 — Tune in this exact order

**1. Base Speed first, with Kp/Kd left at the defaults.**
Start low (60–80). Run it on the straight-ish sections of your track.
Increase Base Speed in steps of 10 until the robot starts missing corners
or wobbling. Then back off by about 20–30% from that point. This is your
working base speed for everything else — don't chase a higher top speed
tonight, chase a speed the robot can actually complete the *whole* track
at, reliably, multiple times in a row.

**2. Kp (turning strength) next.**
- If the robot goes straight through curves and runs off the outside of
  a turn → **increase Kp** (it's not reacting hard enough to the error).
- If the robot zig-zags violently even on straight sections, overcorrecting
  side to side → **decrease Kp**.
Adjust in the small steps the menu gives you (each RIGHT press = +0.02).
Sneak up on it — Kp has a big effect.

**3. Kd (damping) next.**
- If, after fixing Kp, the robot still oscillates/wiggles on straights
  or overshoots then corrects then overshoots the other way → **increase
  Kd** a bit at a time. Kd smooths out the correction so it doesn't
  overshoot.
- If the robot feels sluggish/laggy reacting to corners after increasing
  Kd a lot → back it off slightly.

**4. Search Speed last (this matters a lot for YOUR track).**
Looking at your track photo: there's a dashed-line loop section and
several sharp hairpin turns where the sensors will briefly see no line
at all. Search Speed controls how fast the robot pivots when that
happens.
- Too low → robot barely moves while searching, takes forever to find
  the line again, looks "stuck" at hairpins.
- Too high → robot overshoots past the line before it can react, spins
  past a reacquired line.
- Start around 70–90 and adjust based on what you see specifically at
  the hairpins and the dashed section.

## Track-specific notes (from your photo)

- **Tight hairpins (the loop after the zigzag, and the double-V near
  the end):** these need your Base Speed to be conservative. A robot
  that's "fast enough" on the straights will almost always overshoot a
  hairpin. If it keeps flying off the same corner, that corner — not
  your average speed — is your speed ceiling. Slow down for the whole
  run to survive that one corner.
- **Dashed-line arc:** the robot will lose the line repeatedly for short
  bursts here. As long as it's driving reasonably straight *into* the
  gap (last known error close to 0) it should coast across on momentum
  and the search-pivot behavior. If it consistently veers off during the
  dashed section, lower Base Speed slightly and/or lower Search Speed so
  the pivot is gentler.
- **Zigzag section:** this is the best test for Kp/Kd — if you can get
  it through the zigzag smoothly, the rest of the track is usually
  easier by comparison. Tune primarily using this section.

## Symptom → Fix quick table

| What you see | Likely fix |
|---|---|
| Robot doesn't move / one wheel doesn't spin | Check L298N wiring/power first, not code. Confirm 5–8mm sensor height too. |
| Drives backward or spins in place at start | Flip `LEFT_MOTOR_IS_A`, re-upload |
| Sensors show 0 over black line | Flip `SENSOR_ACTIVE_HIGH`, re-upload, or re-adjust HW-871 trimmers |
| Wiggles/zig-zags on straight line | Lower Kp, or raise Kd slightly |
| Cuts corners / drives straight off a curve | Raise Kp, or lower Base Speed |
| Oscillates then overshoots repeatedly | Raise Kd |
| Sluggish, late to react in sharp turns | Lower Kd a little, or raise Kp a little |
| Stalls / buzzes but doesn't move at low speed | Raise Base Speed, or raise `MIN_MOVE_PWM` in code (default 55) |
| Flies off track at ONE specific hairpin | Lower Base Speed overall — that corner is your real speed limit |
| Gets lost / spins wildly in the dashed section | Lower Search Speed |
| Works on bench but fails on the real track | Re-check battery charge and re-check sensor trimmers under the venue's lighting |

## Before each competition run

1. Fresh/charged battery in.
2. Quick SENSOR TEST check (hold LEFT+RIGHT at boot) — venue lighting can
   differ from where you tuned.
3. Place robot square at START, pointing straight along the line.
4. Hold MID, let go immediately, don't nudge it.
5. If it fails a run, change ONE number before the next attempt — not
   three at once. You won't be able to tell what fixed (or broke) it
   otherwise.

Good luck tomorrow.
