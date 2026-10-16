# AI Racing League — Final Project: Autonomous Driver for the Corkscrew Circuit

**Author:** Sriharshini Manda (individual project)
**Driver:** `snakeoil3_gravity_refine_v12.py`
**Track:** Corkscrew (TORCS Road Tracks) · one lap, standing start

| Final result | |
|---|---|
| Lap time | **01:48:28** (1 min 48.28 s) |
| Car damage | **0** |
| Top speed | **239 km/h** |

---

## 1. Problem statement

The challenge is to drive a TORCS race car around the **Corkscrew** circuit with a Python client and to complete **one lap from a standing start as fast as possible with zero car damage**. The client talks to the simulator's SCR server over UDP at 50 Hz: every 20 ms it receives sensor readings (speeds, heading error, lateral position, 19 range-finder rays, engine speed, gear, wheel speeds, distance along the lap) and must answer with steering, throttle, brake and gear commands.

The Corkscrew is hard because it combines very different demands in one lap: long straights where the car reaches the rev limiter (about 235 km/h), a dozen bends of different severity, and **Turn 8** — a blind crest followed by a roughly five-storey drop through a left-then-right S-bend near 2,437–2,531 m of the 3,608 m lap. A driver tuned for the straights crashes at the Corkscrew; a driver tuned for the Corkscrew gives away time everywhere else.

The delivered driver, `snakeoil3_gravity_refine_v12.py`, is the end of a measured iteration series: the clean lap was reduced from 4:01.80 (the first safe baseline) to 01:48:28 by diagnosing each run from the screen recording and the telemetry log, and changing one thing at a time in a new file so that the previous best was never overwritten.

---

## 2. Repository contents

```
ai-racing-submission/
├── README.md                              this file
├── SUBMISSION.md                          submission e-mail text
├── snakeoil3_gravity_refine_v12.py        the final driver (single, self-contained file)
└── evidence/                              telemetry CSV, screenshots (see Section 6)
```

**Required local files:** only the driver. The `snakeoil` UDP client (packet parsing, socket handling) is included in the same file, so there are no local modules, configuration files or assets to copy.

**External dependencies:** none. The driver imports only Python standard-library modules: `socket`, `sys`, `getopt`, `os`, `time`.

---

## 3. Setup and run

### Environment used

| | |
|---|---|
| OS | Windows 10 (build 19045) |
| Python | 3.12.6 |
| Simulator | TORCS (Windows classroom kit, `wtorcs.exe`) with the SCR server driver `scr_server 1` — kit version: not recorded |
| Network | UDP, `localhost`, port 3001 (the driver's defaults) |

### Steps

1. **Start the simulator first.** Open TORCS (`wtorcs.exe`), choose **Race → Practice** (or the kit's quick-race entry), **Configure Race**, select the road track **Corkscrew**, set the only driver to **`scr_server 1`**, one lap, then **New Race**. TORCS shows its waiting screen for a client on port 3001.
2. **Open a terminal in the folder that contains the driver** (`ai-racing-submission/`).
3. **Check syntax (optional):**
   ```
   python -m py_compile snakeoil3_gravity_refine_v12.py
   ```
4. **Run the driver:**
   ```
   python snakeoil3_gravity_refine_v12.py
   ```
   On Windows `py snakeoil3_gravity_refine_v12.py` is equivalent. The defaults are host `localhost`, port 3001, id `SCR`; they can be changed with `-H`, `-p`, `-i` (run with `--help`).
5. The car launches from the standing start and completes the lap. Stop the client with `Ctrl+C` afterwards.

### What the run produces

Every fifth simulation step the driver appends one row to a timestamped CSV in the current folder, `gravity_refine_v12_log_<YYYYMMDD_HHMMSS>.csv` (distance, speeds, angle, lateral position, steering, throttle, brake, gear, rpm, forward radar, target speed, Turn 8 phase, lap timer, profile-assist speed). Only one client may be connected at a time.

---

## 4. Final results

| Item | Value |
|---|---|
| Track / start | Corkscrew, one lap, standing start, `scr_server 1` |
| Completed lap time | **01:48:28** |
| Car damage | **0** |
| Top speed | **239 km/h** (telemetry log: 238.69 km/h; the log samples every 0.1 s and reads 108.27 s at the finish line) |
| Average speed | ≈ 120 km/h over 3,608.45 m |
| Largest lateral offset | `|trackPos|` ≤ 0.96 everywhere (track edge = 1.0) |
| Gear A-B-A flapping | none in the lap |
| Repeat runs | not recorded; each version's time is a single recorded run |

Progression of the clean lap (each time is from a recorded run):

| Driver | Lap time | Main change |
|---|---|---|
| Baseline v1 (README of the first submission) | 04:01:80 | safe stepwise speed caps |
| Tuned v2 | 03:29:77 | continuous, radar-based speed schedule |
| `gravity v3` (best of the experimental family) | 02:24 | — |
| `refine_v2` | 02:17:55 | position-gated Turn 8 plan, hairpin approach cap, commanded-gear logic |
| `refine_v3` | 02:06:29 | consistent gearbox thresholds |
| `refine_v7` | 01:54:96 | speed-scheduled steering gain |
| `refine_v9` | 01:49:57 | per-corner floors, Turn 8 approach |
| `refine_v10` | 01:48:47 | Turn 8 crawl 54 → 57 km/h |
| **`refine_v12` (this submission)** | **01:48:28** | bounded speed-profile assist, torque-based shift points |

---

## 5. Key enhancements

### 5.1 Turn 8 position-gated plan

**Problem.** An earlier attempt to go faster after Turn 8 used a state machine triggered by `speedZ < -1.9`. SCR reports `speedX/Y/Z` in **km/h**, not m/s, so the threshold was a sink rate of only ≈ 0.5 m/s — a hair-trigger. Latched, it fired on a small blip around 450 m, ran the machine to "done" about 2 km before the Corkscrew, and the car entered Turn 8 at 68–73 km/h instead of ≈ 54 and hit the wall near 2,480 m. Its "is the car straight yet" test also passed on every straight of the lap, so it could not detect the end of the S-bend.

**Solution.** The plan is decided only by `distFromStart`, never by a latched sensor event:

| Phase | Distance | Behaviour |
|---|---|---|
| `V3` | elsewhere | normal radar speed map |
| `T8_FORCE` | 2,380 → 2,525 m | crawl at **57 km/h** with firm braking and the proven steering gain; on the flat straight 2,400–2,440 m the cap may rise to 80 km/h and falls back to 57 km/h at the S-bend |
| `T8_WAIT` | 50 steps (1 s) after the S-bend | crawl still forced |
| `T8_RAMP` | until 3,080 m | cap rises 0.9 km/h per step to 150 km/h; **abort rule**: if `|angle| > 0.35` or `|trackPos| > 0.90`, the cap drops to `speed − 10` and is frozen for 50 steps |

**Why the numbers.** The S-bend ends at 2,525–2,531 m in the telemetry; the crawl speed was chosen from a lateral-acceleration estimate (`a ≈ v²·tan(0.366·steer) / (3.0·g)`, fitted to the logs): the S-bend needs ≈ 1.84 g at 57 km/h, compared with ≈ 2.0–2.3 g in corners that held without sliding, while the original crash corresponds to ≈ 2.6–3.0 g. The ramp end was moved from 3,010 to 3,080 m after the legacy plunge clamp re-fired at 3,010 m and braked the car from 120 to 54 km/h. Kick-down and steering gain inside the crawl keep the behaviour that had been proven in earlier laps.

**Result.** The Corkscrew is taken with a peak `|trackPos|` of 0.92 and zero damage, and the descent after it, which the earlier gravity-v3 driver took at 54 km/h, now runs at up to 150 km/h.

### 5.2 Soft centerline steering (speed-scheduled lane-keeping gain)

**Problem.** In the 2:00 lap the car rode the edge of the track in every tight bend (`trackPos` −1.02 at 400–500 m, −1.19 at 1,900–2,000 m, −0.90 at the hairpin) although the steering command stayed below 0.3 of full lock, so the tyres were not saturated. The cause is the structure of the steering law: with only proportional position feedback, a lane keeper settles at a constant lateral offset in a constant-radius bend,

```
trackPos_steady = steer_needed · stabilizer / kp
```

so the car sat on the outside edge, short of radar look-ahead and short of margin, and every corner floor had to be kept low.

**Solution.** The steering law is

```
steer_raw = 0.85·angle − ( kp·trackPos + 3.2·Δ trackPos )
steer     = steer_raw / max(1, 1 + 0.016·(speed − 35))     above 60 km/h
          = 1.15 · steer_raw                                below 60 km/h
kp        = 0.60 up to 80 km/h, falling linearly to 0.36 at 130 km/h and above
            (0.36 inside the Turn 8 crawl)
```

A higher `kp` gives a "soft" pull back to the centreline in corners; at high speed it returns to the lower gain, where a higher gain would ring.

**Technical justification.** A kinematic lane-keeping model (wheelbase 3.0 m, steering lock 0.366 rad, actuator lag 0.1 s) driven by the controller's *exact* steering law reproduces the measured offsets without further fitting: 68 km/h bend model −0.93 / peak −1.02 against log −0.95 / −1.02; 76 km/h bend model +0.64 against log +0.62; hairpin model −1.09 against log −0.90. In the model an **integral** term winds up and oscillates after each corner exit (rejected), a flat `kp = 0.60` rings above 150 km/h, and the **scheduled** gain removes about 35 % of the offset without the ringing.

**Result.** Measured peak offsets fell from 1.02 / 1.19 / 0.90 to 0.80 / 0.94 / 0.63, as predicted, and the saved margin was spent on faster corners: the lap fell by **5.06 s in a single iteration** (2:00.02 → 1:54.96), the largest step after the gearbox fix.

### 5.3 Safe speed-profile assist

**Idea.** The radar sees the exit of a corner, or the wall of a gentle bend, only late, so the car brakes early and accelerates late. A position-indexed table of speeds, built from the lap itself, can *raise* the target where the radar is too cautious: `target = max(radar_target, min(table, radar_target + 40 km/h))`.

**What failed first.** Version 11 built the table from the steering of the previous lap. Because the car steers reactively, the curvature estimate rose 10–25 m after the true corner entry; combined with `max(radar, table)` it removed the radar map's early braking. The car arrived at three bends 20–40 km/h faster than it had ever been proven to take them (127 km/h at ≈ 746 m against 108 km/h; 137 km/h at ≈ 1,015 m; 164 km/h at ≈ 1,933 m) and left the road each time; the reverse-escape routine then cost about 10 s.

**The safe version (v12).**

1. The curvature estimate looks **30 m ahead** (running maximum over `[s−6, s+30]`), so braking starts before the steering does.
2. The table brakes at **15 m/s²**, below the ≈ 20 m/s² the proportional brake actually delivers.
3. Every table value is capped at the speed **the proven v10 lap drove there plus 40 km/h** on free road, but only **+8 km/h within 90 m of a corner**.
4. A runtime **radar guard**: no boost when the wall is closer than 0.8 s ahead.
5. Switched off from 2,370 to 3,100 m (Turn 8 and the long downhill keep their hand-tuned plan) and whenever the car is off its line (`|trackPos| > 0.90`, `|angle| > 0.30` or `|speedY| > 15 km/h`); the Turn 8, hairpin and bend caps are applied after it, so they still clamp it.

**Verification.** Replaying the 575 logged states of the crashed lap through the code, v12 issues the same brake commands as v10 at 746, 1,015 and 1,933 m, where v11 braked about half as hard, and brakes less than v10 in only 9 steps, all in the gentle bends at 586–640 m, 1,243 m and 1,617 m where the table is meant to act. In the real run v12 entered the same bends at 100 / 100 / 104 / 100 km/h (v10: 108 / 100 / 106 / 98 km/h) with the same peak `|trackPos|` (0.76 / 0.73 / 0.96 / 0.88 against 0.77 / 0.73 / 0.94 / 0.91): the crash is gone. v12 also changed the upshift points to 11,500 rpm (11,800 uphill), derived from a relative torque curve measured in 6th gear. The two changes were run together and gained 0.44 s over v10; their individual contributions were not measured.

---

## 6. Evidence

Place the following in `evidence/` before committing (see the instructions given with this package):

| Item | Notes |
|---|---|
| `gravity_refine_v12_log_20261014_221322.csv` | the real telemetry log of the recorded lap (2.2 MB, one row per 5 steps) |
| PNG screenshots | e.g. the finished race showing lap time and zero damage, and the TORCS run in progress |
| Demonstration video | **do not commit the MP4** (63 MB; the course guidance asks to keep large videos out of the repository) — use the link below |

**Demonstration video:** https://drive.google.com/file/d/1AnSqm8uMnyo1-LQUDmnEmm1W6EgikYjb/view?usp=sharing

---

## 7. Limitations

* **Single recorded run.** The lap time is one run; repeats were not recorded. The simulation appears deterministic (stretches that were not changed reproduced their times to within 0.01–0.1 s), but this was not tested systematically.
* **Damage is read from the recording**, not logged: the telemetry CSV has no damage column.
* **Telemetry is sampled every 5 steps (0.1 s)**, so very short events can be missed and the lap time in the log differs from the simulator's by about 0.01 s.
* **Track-specific.** The Turn 8 plan, approach caps, corner floors and the speed table are indexed by distance along the Corkscrew; another track needs new zones and a new table.
* **Models are approximations.** The kinematic and lap-time models were fitted to logs and are used to rank corners and compare designs; the lateral-acceleration figures are estimates, not measured tyre grip. An offline lap model puts an upper bound of roughly 1:39 on this driver's current line, which would need ≈ 2 g in every corner.
* **Weak off-track recovery.** The stuck routine needs near-zero speed; a car spinning in gravel at ±20 km/h does not trigger it.
* **The speed table is bounded by what the car has already driven**, so it adds little (0.44 s together with the shift points) — it was deliberately made safe rather than aggressive.
* **Tested only on Windows 10 with the classroom TORCS kit**; TORCS and kit version numbers were not recorded.

---

## 8. Credits and licensing

The `snakeoil` UDP client library is by Chris X Edwards; TORCS and the SCR server patch belong to their authors; the `gym_torcs` support files and challenge kit were provided by IBM and are not part of this submission. The driver logic in `snakeoil3_gravity_refine_v12.py` builds on the provided client; no ownership is claimed over the provided code.
