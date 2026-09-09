# JUKTIrekha V1.0

A line following robot built by members of **JUKTI — Club of CSE, IUB**.

JUKTIrekha runs a PID line-following controller on an Arduino Nano with a five-channel digital IR array. Everything is tunable at runtime over a serial console — no recompiling to change a gain — and settings persist in EEPROM across power cycles.

---

## Contents

- [Hardware](#hardware)
- [Wiring](#wiring)
- [Getting started](#getting-started)
- [Serial console](#serial-console)
- [Tuning guide](#tuning-guide)
- [How it works](#how-it-works)
- [Track features](#track-features)
- [Troubleshooting](#troubleshooting)
- [Team](#team)

---

## Hardware

| Component | Notes |
|---|---|
| Arduino Nano | ATmega328P (Old Bootloader) |
| L298N motor driver | ENA/ENB jumpers removed for PWM speed control |
| N20 DC gearmotors ×2 | Differential drive |
| HW-871 5-channel IR array | Digital output, fixed hardware threshold |
| 12V battery | 2200mAh LiPo |

### A note on the HW-871

This board outputs **digital** levels with a threshold fixed in hardware — there are no trimmer potentiometers. It reads HIGH over white and LOW over black, and also LOW when nothing is within detection range.

Because the threshold cannot be adjusted, **ride height is the primary lever**. Aim for roughly 3-5 mm above the surface. Too high and every channel reads as out-of-range; not low enough and not high enough results in both black and white saturate to the same reading.

Line surface matters just as much. Glossy black tape reflects infrared almost as well as white paper and will not register as a line no matter how the code is configured. Use matte black tape or black paper.

---

## Wiring

### Motor driver

| L298N | Nano |
|---|---|
| ENA | D10 (PWM) |
| IN1 | D8 |
| IN2 | D7 |
| ENB | D9 (PWM) |
| IN3 | D4 |
| IN4 | D2 |
| GND | GND (must be common with the Nano) |

### Sensor array

| HW-871 | Nano |
|---|---|
| OUT1 (leftmost) | A0 |
| OUT2 | A1 |
| OUT3 (centre) | A2 |
| OUT4 | A3 |
| OUT5 (rightmost) | A4 |
| VCC | 5V |
| GND | GND |

### LEDs

| LED state | Meaning |
|---|---|
| Slow blink | Idle, waiting for `G` |
| Solid on | Running, line detected |
| Fast blink | Line lost — crossing a gap or searching |

---

## Getting started

1. Open `JUKTIrekha_V1.0.ino` in the Arduino IDE and upload to the Nano.
2. Open Serial Monitor at **115200 baud**, with the line ending set to **New Line**.
3. Type `?` and press Enter. The command list and current settings should print.
4. Send `C` and slide the sensor array across the line by hand. You should see a single `#` walking left to right:

```
  [..#..]  count=1  pos=0
  [.#...]  count=1  pos=-1000
```

If the pattern never changes, stop and fix the hardware before going further — check ride height, then the line surface, then the polarity setting (`V`).

5. Send `M` to run the motor balance test, and set the trim (see below).
6. Tune P and D on a straight section, then `W` to save.

---

## Serial console

| Command | Effect |
|---|---|
| `G` | Start following |
| `X` | Stop |
| `C` | Live sensor check, 15 seconds |
| `M` | Motor balance test |
| `P <val>` | Set Kp |
| `I <val>` | Set Ki |
| `D <val>` | Set Kd |
| `S <val>` | Base speed, 0–255 |
| `A <val>` | Max speed clamp |
| `N <val>` | Minimum PWM (stall compensation) |
| `B <val>` | Motor trim, −1 to +1 |
| `V <0\|1>` | Line polarity, 1 = line reads LOW |
| `Q <ms>` | Blind coast time across a gap |
| `F <0–0.8>` | Corner slowdown factor |
| `T <0–3>` | Telemetry: off / human / plotter / raw bits |
| `W` | Save to EEPROM |
| `L` | Load from EEPROM |
| `R` | Restore defaults |
| `?` | Help and current settings |

Telemetry mode `T 2` outputs tab-separated values suited to the Arduino Serial Plotter: position, correction, left PWM, right PWM.

---

## Tuning guide

### Motor balance

N20 gearmotors are rarely matched. Send `M` — the robot drives both motors forward at equal PWM for 2.5 seconds.

- Curves **right** → left motor is stronger → `B 0.08`
- Curves **left** → right motor is stronger → `B -0.08`

Adjust in steps of 0.02 and repeat until it tracks straight. Typical values fall between 3% and 10%.

If one wheel does not turn at all at low speed, raise the minimum PWM with `N 55` or higher until both motors start reliably.

### PID

Tune on a **straight** section, not a curve. Order matters — each term is judged by a symptom the previous one creates.

**1. Isolate P.**

```
S 90
D 0
I 0
P 0.10
G
```

Ask one question only: does it stay on the line? If not, raise Kp (0.14, 0.18, 0.22) until it does. Ignore any wobble at this stage.

**2. Add D.** Start at roughly 10× your Kp and climb: `D 1.5`, `D 2.2`, `D 3.0`. The wobble should shrink each step. Stop when it's gone, or back off one step if the robot starts vibrating.

**3. Leave I at zero.** Only add a small Ki (0.0002) if the robot tracks well but consistently rides to one side — and check the motor trim first, since a persistent lean is more often mechanical.

**4. Raise the speed.** Increase `S` gradually. Each speed increase usually needs slightly more Kd.

Save with `W`.

### Symptom reference

| Symptom | Adjustment |
|---|---|
| Drifts off gentle curves | Kp up |
| Snakes side to side | Kd up, or Kp down |
| Twitchy or buzzing | Kd down |
| Overshoots sharp corners | Kd up, `F` up |
| Rides one edge consistently | Check trim, then small Ki |
| Slowly swings wider over time | Ki too high, set to 0 |
| Fine slow, fails fast | Lower `S`, raise `F` |

---

## How it works

Five sensors report a binary on-line/off-line state each cycle. A weighted average of the active sensor indices produces a **position** value from −2000 (line at far left) to +2000 (line at far right), with 0 meaning centred.

That position becomes the PID error term. The correction is added to one motor and subtracted from the other, steering the robot back toward centre. The control loop runs at 200 Hz.

Because the sensors are digital, position takes discrete steps of 500 rather than varying smoothly. This makes the error signal jumpy, which is why Kd values here run higher than typical analog-sensor builds.

**Corner slowdown** scales base speed down as the error grows:

```
speed = baseSpeed × (1 − cornerSlow × |error| / 2000)
```

At `F 0.35`, the robot runs at full speed when centred and 65% when the line reaches an outer sensor. This keeps straight-line speed high without sacrificing tight turns.

**Gap handling** applies when all five sensors read white. Rather than stopping or spinning, the robot holds its last steering correction for `gapMs` milliseconds. A straight stays straight and a curve keeps curving, which carries it across dashed sections. Only after that window expires does it treat the loss as real and pivot to search for the line.

---

## Track features

**Crossings and intersections** need no special handling. When all five sensors read black, the weighted average lands on exactly 0 — centred — so the robot drives straight through. This falls out of the maths for free.

**Dashed sections** are handled by the gap logic. Tune `Q` empirically at the speed you actually run: if the robot exits a gap too early and loses the line, raise it; if it drives off the track on a genuine loss, lower it. 500 ms is a reasonable start.

**Tight loops** where the line passes close to itself are the hardest case for a five-channel array, since the sensors can latch onto a neighbouring segment. Lower speed and lower ride height both help. Test these sections in isolation.

---

## Troubleshooting

**All sensors read the same value regardless of surface.** Check ride height first, then the line surface — glossy tape is the most common culprit. Confirm the board has 5 V and that the IR emitters are lit (point a phone camera at them; working emitters glow faint purple-white on screen).

**Serial input does nothing.** Set the line ending dropdown to New Line. The console also accepts commands after a brief typing pause as a fallback, and echoes everything it receives as `rx> …`.

**Robot ignores Kp entirely.** The position value is probably wrong. Run `C` and confirm a single `#` tracks your hand movement. Check that the sensor connector isn't reversed — covering the leftmost sensor should change the *first* value.

**Steering goes the wrong way.** Swap the OUT1/OUT2 pair with OUT3/OUT4 on the L298N, or swap the `ENA`/`IN1`/`IN2` defines with `ENB`/`IN3`/`IN4`.

**Motors don't move at all.** Verify the ENA/ENB jumpers are off, the Nano and L298N share a ground, and the battery holds voltage under load rather than at rest.

**Settings behave oddly after an update.** The EEPROM magic number changes between versions so old data is ignored. Send `R` then `W` to write a clean set of defaults.

---

## Team

Built by members of **JUKTI — Club of CSE, IUB** at Independent University, Bangladesh.

- [Raiyan Bin Rais](https://www.github.com/mikealvarez9999)
- [Asif Amin](https://github.com/Strilitxia)
- [Md Saad Islam](https://github.com/saadaintsad)
- [Md Aman Ullah Al Mubin](https://github.com/amanullahalmubin)

---

## Repository

```
JUKTIrekha_V1.0/
└── JUKTIrekha_V1.0.ino
```

No external libraries required beyond `EEPROM.h`, which ships with the Arduino IDE.
