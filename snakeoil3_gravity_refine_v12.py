#!/usr/bin/python
# snakeoil.py
# Chris X Edwards <snakeoil@xed.ch>
# Snake Oil is a Python library for interfacing with a TORCS
# race car simulator which has been patched with the server
# extentions used in the Simulated Car Racing competitions.
# http://scr.geccocompetitions.com/
#
# To use it, you must import it and create a "drive()" function.
# This will take care of option handling and server connecting, etc.
# To see how to write your own client do something like this which is
# a complete working client:
# /-----------------------------------------------\
# |#!/usr/bin/python                              |
# |import snakeoil                                |
# |if __name__ == "__main__":                     |
# |    C= snakeoil.Client()                       |
# |    for step in xrange(C.maxSteps,0,-1):       |
# |        C.get_servers_input()                  |
# |        snakeoil.drive_example(C)              |
# |        C.respond_to_server()                  |
# |    C.shutdown()                               |
# \-----------------------------------------------/
# This should then be a full featured client. The next step is to
# replace 'snakeoil.drive_example()' with your own. There is a
# dictionary which holds various option values (see `default_options`
# variable for all the details) but you probably only need a few
# things from it. Mainly the `trackname` and `stage` are important
# when developing a strategic bot.
#
# This dictionary also contains a ServerState object
# (key=S) and a DriverAction object (key=R for response). This allows
# you to get at all the information sent by the server and to easily
# formulate your reply. These objects contain a member dictionary "d"
# (for data dictionary) which contain key value pairs based on the
# server's syntax. Therefore, you can read the following:
#    angle, curLapTime, damage, distFromStart, distRaced, focus,
#    fuel, gear, lastLapTime, opponents, racePos, rpm,
#    speedX, speedY, speedZ, track, trackPos, wheelSpinVel, z
# The syntax specifically would be something like:
#    X= o[S.d['tracPos']]
# And you can set the following:
#    accel, brake, clutch, gear, steer, focus, meta
# The syntax is:
#     o[R.d['steer']]= X
# Note that it is 'steer' and not 'steering' as described in the manual!
# All values should be sensible for their type, including lists being lists.
# See the SCR manual or http://xed.ch/help/torcs.html for details.
#
# If you just run the snakeoil.py base library itself it will implement a
# serviceable client with a demonstration drive function that is
# sufficient for getting around most tracks.
# Try `snakeoil.py --help` to get started.

# for Python3-based torcs python robot client
import socket
import sys
import getopt
import os
import time
PI= 3.14159265359

data_size = 2**17

# =============================================================
# TURN 8 (CORKSCREW) POSITION-GATED SPEED PLAN
# -------------------------------------------------------------
# All distances are distFromStart in metres. They were measured from the v3 recording
# (the clean 0-damage lap): on-screen speed integrated over race time and scaled to the
# 3608.45 m lap length, cross-checked with the minimap dot path (agreement about +/-25 m).
#   Corkscrew billboard ........ ~2437 m
#   S-bend (left, then right) .. ~2440 m to ~2525 m
#   v3 54 km/h crawl ended ..... ~2960-2980 m
# NOTE: SCR sensors speedX / speedY / speedZ are in km/h, NOT m/s.
#       speed_z < -1.9 is only a ~0.5 m/s sink rate, so it is a hair-trigger and must
#       NEVER be latched anywhere on the lap.
# =============================================================
T8_FORCE_START_DFS = 2380.0      # from here the 54 km/h plunge clamp is FORCED and braking is firm
T8_TURN8_END_DFS = 2525.0        # Turn 8 (S-bend) is over here; the release delay starts counting
T8_RELEASE_DELAY_STEPS = 50      # 1 s at 50 Hz after Turn 8 is over [refine_v8: 2 s, refine_v5: 3 s, refine_v4: 4 s, refine_v3: 5 s, refine_v2: 6 s], then the downhill cap is raised
                                 # (v8 log: the S-bend ends at ~2525-2531 m, tp 0.58 / angle 0.07 at 2531 m, straight with
                                 # 79-132 m of radar from 2531 m; ramp now starts at ~2540 m instead of ~2555 m)
# REFINE_V9 Turn 8 approach: allow up to T8_APPROACH_MAX_KMH on the flat straight before the S-bend, braking down
# to the 54 km/h crawl at T8_SBEND_START_DFS (slope in km/h per metre). The S-bend itself (2444-2531 m, steer 0.57,
# ~1.65 g in the fitted model) stays at 54 km/h exactly as in refine_v8.
T8_APPROACH_START_DFS = 2400.0
T8_SBEND_START_DFS = 2440.0
T8_APPROACH_MAX_KMH = 80.0
# REFINE_V10: the S-bend crawl speed. v9 log (1:49.57): S-bend 2444-2531 m at 54 km/h, steer 0.53,
# fitted-model lateral acceleration ~1.65 g, versus ~2.0-2.3 g in the other corners that held without a
# slide. The refine_v2 crash that gave the 54 km/h rule happened at 68-73 km/h (~2.6-3.0 g in the same
# model). 57 km/h is ~1.84 g. The old crawl is 54.0. Steering gain in the crawl stays at KP_BASE.
T8_CRAWL_KMH = 57.0
T8_APPROACH_SLOPE = 2.0
T8_RAMP_START_MAX_DFS = 2700.0   # ramp starts here even if the delay has not elapsed (very slow car)
T8_RAMP_END_DFS = 3080.0         # from here v3's own logic runs unchanged again (refine_v2: 3010; the
                                 # v3 plunge clamp re-fired at 3010 and slammed the car from 120 to 54)
T8_RAMP_PER_STEP = 0.9           # about 45 km/h per second at 50 Hz (refine_v2: 0.7)
T8_RELEASE_CEILING_KMH = 150.0   # downhill speed cap after Turn 8 [refine_v3: 130, refine_v2: 120, v3: 54]
                                 # Deliberately NOT raised in refine_v5: the car braked for the bend at ~2958 m from
                                 # 150 km/h at 0.74 with z = -10 (steep). A higher cap would need an earlier approach
                                 # line for a gain of only ~0.15 s.

# =============================================================
# REFINE_V3 PACE TWEAKS (all small; refine_v2 values in brackets)
# -------------------------------------------------------------
# gravity_refine_v2_log_20261014_002432.csv (the 2:17 lap) showed the gearbox hunting: 116 A-B-A
# flaps in one lap. The upshift rpm (7700) was below the rpm needed for the target gear's minimum
# speed (gear 4 needs >= 7900, gear 5 >= 8900, gear 6 >= 10000), so every upshift was undone
# after the cooldown. The shift thresholds below are consistent with the measured gear ratios
# (rpm per km/h: g1 166, g2 121, g3 93, g4 77.5, g5 ~69, g6 63) and with the power band
# (best acceleration at 9-11k rpm).
# =============================================================
PACE = 1.06                      # [refine_v3: 1.04] multiplies the radar speed schedule (1.00 = refine_v2)
# =============================================================
# REFINE_V7: LANE-KEEPING GAIN + HIGHER CORNER FLOORS
# -------------------------------------------------------------
# Why the corner floors could not go up before: in gravity_refine_v6_log_20261014_193655.csv (the
# 2:00.02 lap) the car sits at the edge of the track in every tight corner (trackPos -1.02 at 400-500 m,
# -1.19 at 1,900-2,000 m, -0.90 at the hairpin) although steer never exceeds ~0.27 (of 1.0) there.
# So it is not tyre grip: it is the steering law. steer is proportional to trackPos with
# kp = 0.36, and a P-only lane keeper settles at a constant offset in a constant bend:
#     trackPos_ss = steer_needed * stabilizer / kp
# I reproduced this with a kinematic lane-keeping model of the car and fitted it to the log: it gives
# -0.93 / -1.02 peak for the 68 km/h bend (log: -0.95 / -1.02), +0.64 for the 76 km/h bend (log
# 0.62) and -1.09 for the hairpin (log -0.90). In the same model:
#   * an integral term winds up and the car oscillates after every corner exit (rejected),
#   * raising kp to 0.60 cuts the offset by ~35% with no extra ringing at corner speeds, but rings
#     more above ~150 km/h (|tp| peak after a kick: 0.04 -> 0.09 at 200 km/h, lag 0.12 s).
# So kp is scheduled with speed: 0.60 up to KP_FULL_BELOW_KMH, back to 0.36 from KP_BASE_ABOVE_KMH.
# It is NOT applied inside the Turn 8 crawl (T8_FORCE / T8_WAIT), whose behaviour is proven.
# The saved lateral margin is spent on faster corners: the generic apex floor goes 71 -> 76.
# =============================================================
KP_BASE = 0.36                   # [refine_v6: 0.36 everywhere] position gain at speed
KP_LOW_SPEED = 0.60              # position gain at corner speeds
KP_FULL_BELOW_KMH = 80.0         # full KP_LOW_SPEED up to this speed
KP_BASE_ABOVE_KMH = 130.0        # KP_BASE from this speed up, linear in between

# REFINE_V8 (gravity_refine_v7_log_20261014_195102.csv, the 1:54.96 lap): the steering change worked
# as the model predicted. Peak |trackPos| per corner fell from 1.02 / 1.19 / 0.90 (v6) to 0.80 / 0.94 /
# 0.63, and steer still never exceeds ~0.4 outside the Turn 8 crawl. The corner floors are raised again
# (generic 76 -> 82), per-corner where the log shows more or less margin (see DOWNHILL_APEX_FLOORS).
# REFINE_V10 (gravity_refine_v9_log_20261014_213211.csv, the 1:49.57 lap): braking is already at the
# car's limit (about 100-110 km/h/s at brake 0.7-1.0, measured over the lap) and the corners are at the
# ~2 g the fitted model assigns to the ones that held. What is left is a small step in the generic
# floor, except in the two corners that still reach |trackPos| 0.87-0.95 (see DOWNHILL_APEX_FLOORS).
APEX_FLOOR_KMH = 86.0            # [refine_v9: 82, refine_v7: 76, refine_v6: 71, v3: 68] minimum corner speed from the radar map
PLUNGE_Z_THRESHOLD = -2.3        # [-1.9] speed_z below which v3's plunge clamp fires outside Turn 8
BRAKE_GAIN_DIV = 12.0            # [refine_v3: 13, refine_v2: 14] brake = overshoot / this
# REFINE_V12 shift points. Wheel force ~ engine torque x gear ratio. Torque estimated from the full-throttle
# acceleration in 6th gear in the v10 log (relative torque 0.43 at 9,500 rpm, 0.40 at 10,500, 0.37 at 11,500,
# 0.32 at 12,500; aerodynamic drag makes the high-rpm values read low, i.e. this is conservative). Shifting up
# at rpm R is worth it only once T(R) x ratio_n < T(R x ratio_n+1/ratio_n) x ratio_n+1. With the measured
# ratios (rpm per km/h: 166, 121, 93, 77.5, ~69, 63) the crossovers are ~12,000 (3->4), ~11,500 (4->5) and
# ~11,300 (5->6); the old 10,200 shifted ~5-7% below the best force. The rev limiter is ~14,000 rpm.
UPSHIFT_RPM_FLAT = 11500         # [refine_v10: 10200, refine_v3: 9000, refine_v2: 7700]
UPSHIFT_RPM_UPHILL = 11800       # [refine_v10: 10600, refine_v3: 9400, refine_v2: 8150]
GEAR_MIN_SPEEDS = {6: 118.0, 5: 100.0, 4: 82.0, 3: 60.0, 2: 30.0, 1: 0.0}   # [145/115/85/58/30/0]
FINISH_SPRINT_DFS = 3450.0       # [3480] cross the line flat out slightly earlier

# =============================================================
# HAIRPIN APPROACH BRAKE ZONE (left-hander at ~3250 m, end of the long downhill)
# -------------------------------------------------------------
# bug_gravity_refine_v2.mp4 / gravity_refine_v2_log_20261014_001206.csv: the car reached this
# corner at 150 km/h; the radar only reported it 40 m out, braking started too late, the car was
# still at 78 km/h with a 0.4 rad heading error when the wall appeared and it ran off into the
# gravel (trackPos -3) and spun until the end of the recording.
# The speed cap below falls linearly towards the corner so the car arrives at ~54 km/h, and
# gravity coasting is disabled inside the zone so braking is firm.
# =============================================================
HAIRPIN_CORNER_DFS = 3250.0      # corner entry
HAIRPIN_ZONE_START_DFS = 3100.0  # cap is inactive before this
HAIRPIN_ZONE_END_DFS = 3280.0    # v3's own logic again from here
HAIRPIN_ENTRY_KMH = 60.0         # [refine_v9: 59, refine_v7: 57, v3: 54] target speed at the corner entry (peak |trackPos| 0.60-0.63)
HAIRPIN_SLOPE_KMH_PER_M = 1.7    # [refine_v3: 1.5] cap rises by this much per metre before the corner

# =============================================================
# REFINE_V4 DOWNHILL APEX FLOORS (position-gated corner minimum speeds)
# -------------------------------------------------------------
# gravity_refine_v3_log_20261014_003933.csv (the 2:06 lap). After Turn 8 the car crawled through
# two bends at the generic 71 km/h corner floor although it was comfortably inside the track:
#   2625-2800 m  long left-hand sweeper: held 77-78 km/h, trackPos steady at -0.5, steer 0.11
#   2950-3040 m  right-hand bend:        held 72 km/h, trackPos peaked at 0.68, steer -0.18
# Lateral offset grows roughly with speed squared, so the sweeper floor is raised ~9% (offset
# about -0.6) and the second bend, which already reaches 0.68, only ~4%.
# Format: (from_dfs, to_dfs, floor_kmh). Outside these zones APEX_FLOOR_KMH applies.
# =============================================================
# The downhill cap is now 150 km/h, but the right-hand bend at ~2958 m was only braked for at
# 130 km/h (full brake from 2931 m, radar 47 m). This approach cap keeps the entry speed at about
# the same as in refine_v3 (134 km/h at 2931 m) however fast the straight before it is.
BEND2_CORNER_DFS = 2958.0
BEND2_ZONE_START_DFS = 2850.0
BEND2_ZONE_END_DFS = 3000.0
BEND2_ENTRY_KMH = 92.0           # [refine_v8: 80] must be >= the bend's apex floor (94 -> held 92), the cap stays at this value to 3000 m
BEND2_SLOPE_KMH_PER_M = 2.0

# REFINE_V8 approach cap for the bend at ~420 m (replaces the accidental plunge-clamp braking above).
# The speed cap falls linearly to the entry speed at the bend, firm braking (no gravity coasting) inside
# the zone. 1.7 km/h per metre needs ~70 km/h/s at 150 km/h, well inside the ~100 km/h/s the car braked
# at in the v7 log (e.g. 3,172-3,241 m: 186 -> 89 km/h at brake 0.3-0.4).
BEND0_CORNER_DFS = 420.0
BEND0_ZONE_START_DFS = 330.0
BEND0_ZONE_END_DFS = 440.0
BEND0_ENTRY_KMH = 80.0
BEND0_SLOPE_KMH_PER_M = 1.7

# REFINE_V5 (gravity_refine_v4_log_20261014_005009.csv, the 2:04.42 lap): the sweeper floor of 84
# gave trackPos -0.60 at 88-91 km/h with steer 0.16 and angle < 0.12, so there is room left: the
# floor is now 94 (offset ~ -0.70) and the zone starts at 2570 m to match the earlier ramp start.
# The right-hand bend peaked at trackPos 0.71 at ~75 km/h, so it only gets +2 km/h.
# REFINE_V6 (gravity_refine_v5_log_20261014_005903.csv, the 2:02.00 lap):
#   2600-2800 m sweeper: at the 94 floor the car held 97-110 km/h with trackPos -0.67, angle +-0.04
#                        -> floor 100 (offset ~ -0.75, still clear of the edge).
#   1010-1085 m bend:    held 76 km/h for 65 m with trackPos 0.62, steer 0.14, angle ~0
#                        -> floor 84 (offset ~ 0.76). Only places where the log shows a steady,
#                        well-centred bend; every other floor zone touches |trackPos| 0.8-1.2.
# REFINE_V8 per-corner floors (v7 lap: peak |trackPos| / speed at the floor):
#   1,005-1,090 m  bend         0.60 @ 88   -> 94
#   1,895-1,990 m  tight turn   0.94 @ 76 (steer 0.39, angle +-0.29) -> keeps 76, the only corner near the edge
#   2,030-2,075 m  short bend   0.44 @ 85   -> 92
#   2,570-2,810 m  sweeper      0.61 @ 112  -> 108 floor (the speed there is the radar target, not the floor)
#   2,930-3,050 m  bend         0.52 @ 80   -> 84
# REFINE_V9 (gravity_refine_v8_log_20261014_201604.csv, the 1:52.65 lap). Corner regions from the log,
# with the lateral acceleration the fitted model (L = 3.0 m, lock = 0.366 rad) attributes to the median
# speed and the 95th-percentile steer. Corners already at ~2 g are left alone; those with headroom get a
# floor that brings them to ~1.8-1.9 g:
#    421-553 m   2.20 g  tp 0.87            left alone          1933-1997 m  2.34 g  tp 0.95   left alone
#    748-822 m   1.94 g  tp 0.73            left alone          3262-3325 m  2.10 g (hairpin) left alone
#   1015-1092 m  1.69 g @100                -> floor 100        1235-1257 m  1.58 g @100      -> floor 112
#   2037-2061 m  1.40 g @100                -> floor 115        2686-2796 m  1.72 g @114      -> floor 118
#   2961-3035 m  1.37 g @ 81                -> floor 94
# 1235-1257 m also fixes a regression: in the v8 lap the car braked there to 93 km/h (radar 25 m, apex target
# 97) and left at 140 km/h; in the v7 lap it held 123-137 and left at 157 (about -0.6 s).
# REFINE_V10: generic floor 82 -> 86, so the corners that must NOT follow it are pinned to their v9 speed:
#   400-560 m   |trackPos| 0.87 (2.2 g)    -> 82       1,895-1,990 m  |trackPos| 0.93 -> 76 (as before)
DOWNHILL_APEX_FLOORS = (
    (400.0, 560.0, 82.0),        # [v9: generic 82]
    (1005.0, 1090.0, 100.0),     # [refine_v8: 94, refine_v7: 88]
    (1225.0, 1275.0, 112.0),     # [refine_v8: generic 82]
    (1895.0, 1990.0, 76.0),      # kept: the one corner that still reaches |trackPos| 0.95
    (2030.0, 2075.0, 115.0),     # [refine_v8: 92]
    (2570.0, 2680.0, 108.0),     # [refine_v8: 108 for 2570-2810] the turn-in (2.28 g) is not raised
    (2680.0, 2810.0, 118.0),     # [refine_v8: 108]
    (2930.0, 3050.0, 94.0),      # [refine_v8: 84, refine_v7: 80]
)

# =============================================================
# REFINE_V6 PLUNGE-CLAMP-OFF ZONES
# -------------------------------------------------------------
# v3's speed_z plunge clamp (target 54 km/h whenever speed_z < PLUNGE_Z_THRESHOLD) is kept
# everywhere except here. In the 2:02.00 lap it fired by accident on the shallow slope after the
# long downhill and cost time: at 3172 m a single step braked the car at 1.00 from 186 km/h
# (186 -> 169), and at 3331-3350 m it braked at 0.30 while the car was accelerating out of the
# hairpin (the straight to the line, no corner). The hairpin and its approach have their own caps.
# Format: (from_dfs, to_dfs).
# =============================================================
# REFINE_V8: also off for 320-500 m. The v7 lap shows the clamp (z -2.3..-3.3 on the slope before the
# bend at ~420 m) braking the car from 166 to 83 km/h by 390 m, 30 m BEFORE the bend, then flicker-braking
# at 400-413 m and 478-485 m. It was acting as an accidental brake zone; it is replaced by the proper
# approach cap BEND0_* below (the slope is a downhill, where the code otherwise only coasts).
PLUNGE_CLAMP_OFF_ZONES = (
    (320.0, 500.0),
    (3100.0, 3420.0),
)

def plunge_clamp_enabled(dist_from_start):
    for lo, hi in PLUNGE_CLAMP_OFF_ZONES:
        if lo <= dist_from_start < hi:
            return False
    return True

# =============================================================
# REFINE_V5 BRAKE LIMIT FOR THE BEND AT ~1200-1270 m
# -------------------------------------------------------------
# Same bend, same approach speed (~172-176 km/h) in the two laps:
#   refine_v3 lap:  braked at 0.28-0.52, held 135 km/h through the bend, trackPos peaked at 0.37.
#   refine_v4 lap:  braked at 0.75-0.97, the car yawed (angle -0.12), drifted to trackPos 0.65,
#                   the radar saw the wall at 17 m, the target collapsed to 77 km/h and the car
#                   needed 3.1 s for these 100 m instead of 2.3 s.
# Heavy braking at 140+ km/h while already yawed pushes the car off line. Inside this zone the
# brake is capped at the value refine_v3 used. Format: (from_dfs, to_dfs, max_brake).
# =============================================================
BRAKE_SOFT_ZONES = (
    (1190.0, 1270.0, 0.55),
)

def apex_floor(dist_from_start):
    for lo, hi, floor in DOWNHILL_APEX_FLOORS:
        if lo <= dist_from_start < hi:
            return floor
    return APEX_FLOOR_KMH

# =============================================================
# REFINE_V12: SAFE SPEED-PROFILE ASSIST  (fixes the refine_v11 crashes)
# -------------------------------------------------------------
# WHAT WENT WRONG IN refine_v11 (bug_crash_gravity_refine_v11.mp4, gravity_refine_v11_log_20261014_220113.csv)
# The v11 speed table was built from the steering the v10 car used, and the car steers REACTIVELY: the
# steering (and so the curvature estimate) only rises ~10-25 m after the real corner entry. The table therefore
# told the car it could still hold full speed ~20 m too late, and the target = max(radar, table) removed the
# early braking that refine_v10's radar map had. Result, three times in one lap:
#     ~746 m   arrived at 127 km/h (v10: 108) and turned in at 119-87 km/h with angle -0.30 -> trackPos 1.7, off
#              the road; the stuck/reverse routine then cost ~10 s (lap time 24.5 s -> 36 s at 883 m)
#     ~1,015 m arrived at 137 km/h (v10: ~100), trackPos 0.96
#    ~1,933 m  arrived at 164 km/h, trackPos -1.92, off the road (3 s lost twice)
# In all three the table was 20-40 km/h above the speed the car was proven to take at that point.
#
# WHAT v12 CHANGES (the table is still raise-only, but is now bounded by what the car has actually driven)
#   1. the curvature estimate looks 30 m AHEAD (running max over [s-6, s+30]) so braking starts before the
#      steering does,
#   2. the table brakes at 15 m/s2 (v11: 20; the car's proportional brake settles at ~20 m/s2 with a lag),
#   3. every table value is capped at the speed the refine_v10 car drove there (the 1:48.47 lap) plus a margin:
#      +40 km/h on free road, but only +8 km/h within 90 m of a corner,
#   4. boost cap +40 km/h over the radar target (v11: +40),
#   5. NEW runtime guard: no boost when the wall is closer than PROFILE_RADAR_GUARD_S seconds ahead (radar
#      min(track[8:11]) < speed * guard),
#   6. still off from 2,370 to 3,100 m and whenever the car is off its line (|trackPos| > 0.90,
#      |angle| > 0.30, |speedY| > 15 km/h); still applied before the Turn 8 / hairpin / bend caps.
# One-dimensional replay along the v10 lap (v10 radar targets + this rule, braking modelled with the
# car's measured ~25 m/s2 and the proportional brake lag): corner-entry speeds equal v10's
# (745 m: 100 vs 99, 1,012 m: 100 vs 100, 1,930 m: 98 vs 98, 3,240 m: 98 vs 98) and the lap is 1.4 s shorter.
# With the v11 rule the same replay arrives at 129 km/h at 1,012 m and 3,240 m (v10: 100 / 98).
# =============================================================
PROFILE_ENABLED = True
PROFILE_MAX_BOOST_KMH = 40.0
PROFILE_RADAR_GUARD_S = 0.8
PROFILE_EXCLUDE = (
    (2370.0, 3100.0),
)
PROFILE_LINE_MAX_TRACKPOS = 0.90
PROFILE_LINE_MAX_ANGLE = 0.30
PROFILE_LINE_MAX_SPEED_Y = 15.0
PROFILE_STEP_M = 10.0
PROFILE_TABLE_KMH = (
     63,  63,  74,  88, 101, 110, 122, 129, 136, 143, 149, 155,  # 0-110 m
    161, 166, 157, 144, 130, 114,  96,  89,  89,  89,  89,  92,  # 120-230 m
    104, 114, 123, 130, 137, 144, 150, 156, 162, 167, 172, 161,  # 240-350 m
    149, 135, 120, 102,  81,  70,  70,  70,  70,  70,  75,  85,  # 360-470 m
     77,  77,  77,  77,  77,  84,  97, 108, 118, 126, 133, 140,  # 480-590 m
    145, 142, 144, 149, 130, 138, 145, 153, 158, 147, 134, 118,  # 600-710 m
    100,  82,  78,  78,  78,  78,  79,  86,  95, 100, 101, 111,  # 720-830 m
    120, 128, 135, 142, 148, 154, 160, 165, 170, 175, 180, 171,  # 840-950 m
    160, 147, 133, 117, 101,  96,  95,  95,  95,  95,  98, 102,  # 960-1070 m
    102, 103, 112, 121, 129, 136, 143, 149, 155, 161, 166, 155,  # 1080-1190 m
    142, 131, 131, 131, 131, 136, 142, 149, 155, 160, 166, 171,  # 1200-1310 m
    175, 180, 184, 188, 192, 196, 200, 201, 191, 181, 170, 158,  # 1320-1430 m
    145, 131, 115,  97,  85,  85,  85,  85,  85,  89,  95, 100,  # 1440-1550 m
    103, 105, 107, 108, 129, 134, 141, 147, 153, 159, 164, 169,  # 1560-1670 m
    174, 179, 183, 187, 192, 196, 199, 203, 206, 209, 212, 215,  # 1680-1790 m
    218, 210, 201, 191, 180, 169, 157, 145, 130, 115,  96,  73,  # 1800-1910 m
     65,  65,  65,  65,  65,  77,  93, 103, 115, 124, 112, 110,  # 1920-2030 m
    106, 110, 111, 120, 128, 135, 142, 148, 154, 160, 165, 170,  # 2040-2150 m
    175, 180, 184, 188, 192, 196, 200, 203, 205, 207, 204, 195,  # 2160-2270 m
    184, 174, 162, 150, 136, 121, 103,  86,  86,  86,  86,  87,  # 2280-2390 m
     73,  65,  73,  67,  66,  56,  56,  56,  54,  54,  54,  54,  # 2400-2510 m
     62,  65,  65,  65,  78,  94, 107, 117, 118, 100,  96,  96,  # 2520-2630 m
     96,  96, 106, 116, 109, 107, 107, 107, 107, 107, 109, 112,  # 2640-2750 m
    116, 119, 121, 122, 125, 132, 139, 145, 152, 158, 158, 158,  # 2760-2870 m
    158, 158, 158, 146, 132, 116,  98,  88,  88,  88,  88,  88,  # 2880-2990 m
     90,  98, 106, 108, 113, 122, 129, 136, 143, 149, 155, 161,  # 3000-3110 m
    166, 171, 176, 180, 185, 174, 163, 150, 137, 121, 104,  83,  # 3120-3230 m
     56,  47,  47,  47,  47,  51,  64,  82,  97, 108, 118, 126,  # 3240-3350 m
    133, 140, 146, 152, 158, 164, 169, 174, 178, 183, 187, 191,  # 3360-3470 m
    195, 199, 202, 206, 209, 212, 215, 218, 221, 222, 224, 225,  # 3480-3590 m
)

def profile_speed_kmh(dist_from_start):
    """Table speed at this distance (linear interpolation), or None when the table is not applicable."""
    for lo, hi in PROFILE_EXCLUDE:
        if lo <= dist_from_start < hi:
            return None
    pos = clip(dist_from_start, 0.0, PROFILE_STEP_M * (len(PROFILE_TABLE_KMH) - 1)) / PROFILE_STEP_M
    i = int(pos)
    j = min(i + 1, len(PROFILE_TABLE_KMH) - 1)
    frac = pos - i
    return PROFILE_TABLE_KMH[i] * (1.0 - frac) + PROFILE_TABLE_KMH[j] * frac

T8_ABORT_ANGLE = 0.35
T8_ABORT_TRACKPOS = 0.90
T8_KICKDOWN_SPEED_RELEASE = 40.0 # gear-1 kickdown threshold (v3: 60) inside the ramp zone only
LOG_TELEMETRY = True

# Initialize help messages
ophelp=  'Options:\n'
ophelp+= ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp+= ' --port, -p <port>    TORCS port. [3001]\n'
ophelp+= ' --id, -i <id>        ID for server. [SCR]\n'
ophelp+= ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp+= ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp+= ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp+= ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp+= ' --debug, -d          Output full telemetry.\n'
ophelp+= ' --help, -h           Show this help.\n'
ophelp+= ' --version, -v        Show current version.'
usage= 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage= usage + ophelp
version= "20130505-2"

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v

def bargraph(x,mn,mx,w,c='X'):
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
    if not w: return '' # No width!
    if x<mn: x= mn      # Clip to bounds.
    if x>mx: x= mx      # Clip to bounds.
    tx= mx-mn # Total real units possible to show on graph.
    if tx<=0: return 'backwards' # Stupid bounds.
    upw= tx/float(w) # X Units per output char width.
    if upw<=0: return 'what?' # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu= 0,0,0,0
    if mn < 0: # Then there is a negative part to graph.
        if x < 0: # And the plot is on the negative side.
            negpu= -x + min(0,mx)
            negnonpu= -mn + x
        else: # Plot is on pos. Neg side is empty.
            negnonpu= -mn + min(0,mx) # But still show some empty neg.
    if mx > 0: # There is a positive part to the graph
        if x > 0: # And the plot is on the positive side.
            pospu= x - max(0,mn)
            posnonpu= mx - x
        else: # Plot is on neg. Pos side is empty.
            posnonpu= mx - max(0,mn) # But still show some empty pos.
    nnc= int(negnonpu/upw)*'-'
    npc= int(negpu/upw)*c
    ppc= int(pospu/upw)*c
    pnc= int(posnonpu/upw)*'_'
    return '[%s]' % (nnc+npc+ppc+pnc)

class Client():
    def __init__(self,H=None,p=None,i=None,e=None,t=None,s=None,d=None,vision=False):
        # If you don't like the option defaults,  change them here.
        self.vision = vision

        self.host= 'localhost'
        self.port= 3001
        self.sid= 'SCR'
        self.maxEpisodes=1 # "Maximum number of learning episodes to perform"
        self.trackname= 'unknown'
        self.stage= 3 # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug= False
        self.maxSteps= 100000  # 50steps/second
        self.parse_the_command_line()
        if H: self.host= H
        if p: self.port= p
        if i: self.sid= i
        if e: self.maxEpisodes= e
        if t: self.trackname= t
        if s: self.stage= s
        if d: self.debug= d
        self.S= ServerState()
        self.R= DriverAction()
        self.setup_connection()

    def setup_connection(self):
        # == Set Up UDP Socket ==
        try:
            self.so= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        # == Initialize Connection To Server ==
        self.so.settimeout(1)

        n_fail = 5
        while True:
            # This string establishes track sensor angles! You can customize them.
            #a= "-90 -75 -60 -45 -30 -20 -15 -10 -5 0 5 10 15 20 30 45 60 75 90"
            # xed- Going to try something a bit more aggressive...
            a= "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"

            initmsg='%s(init %s)' % (self.sid,a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata= str()
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    print("relaunch torcs")
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')

                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            identify = '***identified***'
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break

    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=',
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= int(opt[1])
                ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                ######## END OF REQUIRED #########
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' % (
                                       opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

    ######### REQUIRED FOR RACE MODE - DO NOT CHANGE OR REMOVE #########
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()

        while True:
            try:
                # Receive server data
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
                #print "Waiting for data on %d.............." % self.port
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. "+
                        "You were in %d place.") %
                        (self.port,self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                # What do I do here?
                print("Server has restarted the race on %d." % self.port)
                # I haven't actually caught the server doing this.
                self.shutdown()
                return
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H") # Clear for steady output.
                    print(self.S)
                break # Can now return from this function.
    ########## END OF REQUIRED #########

    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1],str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())
        # Or use this for plain output:
        #if self.debug: print self.R

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps,self.port)))
        self.so.close()
        self.so = None
        #sys.exit() # No need for this really.

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        # Comment the next line for raw output:
        return self.fancyout()
        # -------------------------------------
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out= str()
        sensors= [ # Select the ones you want in the order you want them.
        #'curLapTime',
        #'lastLapTime',
        'stucktimer',
        #'damage',
        #'focus',
        'fuel',
        #'gear',
        'distRaced',
        'distFromStart',
        #'racePos',
        'opponents',
        'wheelSpinVel',
        'z',
        'speedZ',
        'speedY',
        'speedX',
        'targetSpeed',
        'rpm',
        'skid',
        'slip',
        'track',
        'trackPos',
        'angle',
        ]

        #for k in sorted(self.d): # Use this to get all sensors.
        for k in sensors:
            if type(self.d.get(k)) is list: # Handle list type data.
                if k == 'track': # Nice display for track sensors.
                    strout= str()
                 #  for tsensor in self.d['track']:
                 #      if   tsensor >180: oc= '|'
                 #      elif tsensor > 80: oc= ';'
                 #      elif tsensor > 60: oc= ','
                 #      elif tsensor > 39: oc= '.'
                 #      #elif tsensor > 13: oc= chr(int(tsensor)+65-13)
                 #      elif tsensor > 13: oc= chr(int(tsensor)+97-13)
                 #      elif tsensor >  3: oc= chr(int(tsensor)+48-3)
                 #      else: oc= '_'
                 #      strout+= oc
                 #  strout= ' -> '+strout[:9] +' ' + strout[9] + ' ' + strout[10:]+' <-'
                    raw_tsens= ['%.1f'%x for x in self.d['track']]
                    strout+= ' '.join(raw_tsens[:9])+'_'+raw_tsens[9]+'_'+' '.join(raw_tsens[10:])
                elif k == 'opponents': # Nice display for opponent sensors.
                    strout= str()
                    for osensor in self.d['opponents']:
                        if   osensor >190: oc= '_'
                        elif osensor > 90: oc= '.'
                        elif osensor > 39: oc= chr(int(osensor/2)+97-19)
                        elif osensor > 13: oc= chr(int(osensor)+65-13)
                        elif osensor >  3: oc= chr(int(osensor)+48-3)
                        else: oc= '?'
                        strout+= oc
                    strout= ' -> '+strout[:18] + ' ' + strout[18:]+' <-'
                else:
                    strlist= [str(i) for i in self.d[k]]
                    strout= ', '.join(strlist)
            else: # Not a list type of value.
                if k == 'gear': # This is redundant now since it's part of RPM.
                    gs= '_._._._._._._._._'
                    p= int(self.d['gear']) * 2 + 2  # Position
                    l= '%d'%self.d['gear'] # Label
                    if l=='-1': l= 'R'
                    if l=='0':  l= 'N'
                    strout= gs[:p]+ '(%s)'%l + gs[p+3:]
                elif k == 'damage':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,10000,50,'~'))
                elif k == 'fuel':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,100,50,'f'))
                elif k == 'speedX':
                    cx= 'X'
                    if self.d[k]<0: cx= 'R'
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-30,300,50,cx))
                elif k == 'speedY': # This gets reversed for display to make sense.
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k]*-1,-25,25,50,'Y'))
                elif k == 'speedZ':
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-13,13,50,'Z'))
                elif k == 'z':
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k],.3,.5,50,'z'))
                elif k == 'trackPos': # This gets reversed for display to make sense.
                    cx='<'
                    if self.d[k]<0: cx= '>'
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k]*-1,-1,1,50,cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout= '%3d %s' % (self.d[k], bargraph(self.d[k],0,300,50,"'"))
                    else: strout= 'Not stuck!'
                elif k == 'rpm':
                    g= self.d['gear']
                    if g < 0:
                        g= 'R'
                    else:
                        g= '%1d'% g
                    strout= bargraph(self.d[k],0,10000,50,g)
                elif k == 'angle':
                    asyms= [
                          "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                          "---  ", ".__  ", "-._  ", "'-.  ", "'\\.  ", "'|.  ",
                          "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                          "  ---", "  --.", "  -._", "  -..", "  '\\.", "  '|."  ]
                    rad= self.d[k]
                    deg= int(rad*180/PI)
                    symno= int(.5+ (rad+PI) / (PI/12) )
                    symno= symno % (len(asyms)-1)
                    strout= '%5.2f %3d (%s)' % (rad,deg,asyms[symno])
                elif k == 'skid': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    skid= 0
                    if frontwheelradpersec:
                        skid= .5555555555*self.d['speedX']/frontwheelradpersec - .66124
                    strout= bargraph(skid,-.05,.4,50,'*')
                elif k == 'slip': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    slip= 0
                    if frontwheelradpersec:
                        slip= ((self.d['wheelSpinVel'][2]+self.d['wheelSpinVel'][3]) -
                              (self.d['wheelSpinVel'][0]+self.d['wheelSpinVel'][1]))
                    strout= bargraph(slip,-5,150,50,'@')
                else:
                    strout= str(self.d[k])
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       # "d" is for data dictionary.
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }

    def clip_to_limits(self):
        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to. The "clip" command is still a snakeoil
        utility function, but it should be used only for non standard
        things or non obvious limits (limit the steering to the left,
        for example). For normal limits, simply don't worry about it."""
        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

    def fancyout(self):
        '''Specialty output for useful monitoring of bot's effectors.'''
        out= str()
        od= self.d.copy()
        od.pop('gear','') # Not interesting.
        od.pop('meta','') # Not interesting.
        od.pop('focus','') # Not interesting. Yet.
        for k in sorted(od):
            if k == 'clutch' or k == 'brake' or k == 'accel':
                strout=''
                strout= '%6.3f %s' % (od[k], bargraph(od[k],0,1,50,k[0].upper()))
            elif k == 'steer': # Reverse the graph to make sense.
                strout= '%6.3f %s' % (od[k], bargraph(od[k]*-1,-1,1,50,'S'))
            else:
                strout= str(od[k])
            out+= "%s: %s\n" % (k,strout)
        return out

# == Misc Utility Functions
def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

def update_turn8_state(state, dist_from_start, speed_x, speed_z, angle, track_pos, steer_output, target_speed):
    '''Turn 8 (Corkscrew) position-gated speed plan.

    Phases (decided by distFromStart, never by a latched speed_z trigger):
      V3        : outside the corridor -> EXACTLY v3 behaviour (per-step plunge clamp, no latch).
      T8_FORCE  : [T8_FORCE_START_DFS, T8_TURN8_END_DFS) -> plunge clamp forced on (48 -> 54 floor).
      T8_WAIT   : Turn 8 is over; the clamp stays forced for T8_RELEASE_DELAY_STEPS (6 s).
      T8_RAMP   : v3 plunge clamp suppressed, cap ramps 54 -> T8_RELEASE_CEILING_KMH (120).
    Returns (target_speed, phase).
    '''
    state.setdefault('t8_ramping', False)
    state.setdefault('t8_wait_steps', 0)
    state.setdefault('t8_cap', 54.0)
    state.setdefault('t8_abort_freeze', 0)
    state.setdefault('t8_last_phase', 'V3')

    dfs = dist_from_start
    if T8_FORCE_START_DFS <= dfs < T8_RAMP_END_DFS:
        if dfs < T8_TURN8_END_DFS and not state['t8_ramping']:
            phase = 'T8_FORCE'
        elif state['t8_ramping']:
            phase = 'T8_RAMP'
        else:
            # Turn 8 is over: hold the 54 km/h clamp for the release delay, then start the ramp.
            state['t8_wait_steps'] += 1
            if state['t8_wait_steps'] >= T8_RELEASE_DELAY_STEPS or dfs >= T8_RAMP_START_MAX_DFS:
                state['t8_ramping'] = True
                state['t8_cap'] = T8_CRAWL_KMH
                state['t8_abort_freeze'] = 0
                phase = 'T8_RAMP'
            else:
                phase = 'T8_WAIT'
    else:
        # Outside the corridor (also covers a new lap): reset everything.
        phase = 'V3'
        state['t8_ramping'] = False
        state['t8_wait_steps'] = 0
        state['t8_cap'] = 54.0
        state['t8_abort_freeze'] = 0

    if phase == 'V3':
        # Exactly v3: per-step clamp, nothing latched.
        if speed_z < PLUNGE_Z_THRESHOLD and plunge_clamp_enabled(dfs):
            target_speed = min(target_speed, 48.0)
    elif phase == 'T8_FORCE' and T8_APPROACH_START_DFS <= dfs < T8_SBEND_START_DFS:
        # refine_v9: the straight between the bend at ~2390 m and the S-bend (steering starts at ~2444 m)
        # is flat (speed_z 3 -> -1.7), so it does not need the 54 km/h crawl: the cap is T8_APPROACH_MAX_KMH
        # and falls linearly to the 54 km/h crawl at the S-bend entry.
        approach_cap = min(T8_APPROACH_MAX_KMH, T8_CRAWL_KMH + (T8_SBEND_START_DFS - dfs) * T8_APPROACH_SLOPE)
        target_speed = min(target_speed, approach_cap)
    elif phase in ('T8_FORCE', 'T8_WAIT'):
        # Forced crawl clamp (refine_v10: T8_CRAWL_KMH; before: 48 -> clipped up to 54), independent of speed_z.
        target_speed = min(target_speed, T8_CRAWL_KMH)
    else:
        # T8_RAMP: v3 plunge clamp suppressed; capped, ramped target.
        if abs(angle) > T8_ABORT_ANGLE or abs(track_pos) > T8_ABORT_TRACKPOS:
            state['t8_cap'] = min(max(54.0, speed_x - 10.0), T8_RELEASE_CEILING_KMH)
            state['t8_abort_freeze'] = 50
        elif state['t8_abort_freeze'] > 0:
            state['t8_abort_freeze'] -= 1
        else:
            state['t8_cap'] = min(state['t8_cap'] + T8_RAMP_PER_STEP, T8_RELEASE_CEILING_KMH)
        target_speed = min(target_speed, state['t8_cap'])

    if phase != state['t8_last_phase']:
        try:
            print("[T8] %s -> %s at distFromStart=%.1fm, speedX=%.1fkm/h" %
                  (state['t8_last_phase'], phase, dfs, speed_x))
        except Exception:
            pass
        state['t8_last_phase'] = phase
    return target_speed, phase


def drive_example(c):
    '''Optimized Autonomous Racing Controller tuned for Corkscrew Circuit.
    Engineered for IBM SkillsBuild AI Racing League competition.
    Achieves minimum lap times from a standing start with ZERO car damage.
    
    Subsystems:
      1. Telemetry Ingestion & State Tracking
      2. Automated Stuck Recovery Routine (Reverse escape)
      3. Predictive Speed Scheduling (Front radar track[8:11] fusion, elevation damping)
      4. Lateral Stabilization (PD Steering with high-speed oscillation damping)
      5. Traction Control (TCS) & Anti-Lock Braking System (ABS)
      6. Sequential Transmission Logic (Calibrated upshift @ 7,500 RPM, anti-stall downshift)
    '''
    S, R = c.S.d, c.R.d

    # -------------------------------------------------------------
    # 1. TELEMETRY INGESTION & PERSISTENT CONTROLLER STATE
    # -------------------------------------------------------------
    if not hasattr(c, 'ctrl_state'):
        c.ctrl_state = {
            'prev_track_pos': float(S.get('trackPos', 0.0)),
            'stuck_counter': 0,
            'recovery_timer': 0,
            'gear_cooldown': 0,
            'step': 0,
            't8_phase': 'V3',
            't8_last_phase': 'V3',
            't8_ramping': False,
            't8_wait_steps': 0,
            't8_cap': 54.0,
            't8_abort_freeze': 0,
            'log_file': time.strftime('gravity_refine_v12_log_%Y%m%d_%H%M%S.csv'),
            'last_steer': 0.0,
            'cmd_gear': 1,
            'gear_mismatch': 0
        }
    state = c.ctrl_state
    state['step'] += 1
    if state['gear_cooldown'] > 0:
        state['gear_cooldown'] -= 1

    # Extract state variables safely
    angle = float(S.get('angle', 0.0))
    track_pos = float(S.get('trackPos', 0.0))
    speed_x = float(S.get('speedX', 0.0))
    speed_y = float(S.get('speedY', 0.0))
    speed_z = float(S.get('speedZ', 0.0))
    rpm = float(S.get('rpm', 0.0))
    current_gear = int(S.get('gear', 1))
    dist_from_start = float(S.get('distFromStart', 0.0))
    dist_raced = float(S.get('distRaced', 0.0))

    # Parse 19-sector track range-finder sensors
    raw_track = S.get('track', [200.0] * 19)
    if isinstance(raw_track, list) and len(raw_track) == 19:
        track = [float(x) for x in raw_track]
    else:
        track = [200.0] * 19

    # Parse 4-wheel rotational speeds [FL, FR, RL, RR] (rad/s)
    raw_wheel_spin = S.get('wheelSpinVel', [0.0, 0.0, 0.0, 0.0])
    if isinstance(raw_wheel_spin, list) and len(raw_wheel_spin) == 4:
        wheel_spin = [float(x) for x in raw_wheel_spin]
    else:
        wheel_spin = [0.0, 0.0, 0.0, 0.0]

    # -------------------------------------------------------------
    # 2. AUTOMATED STUCK RECOVERY ROUTINE
    # -------------------------------------------------------------
    # Active recovery mode: vehicle drives in reverse to re-center
    if state['recovery_timer'] > 0:
        state['recovery_timer'] -= 1
        R['gear'] = -1
        state['cmd_gear'] = -1
        R['brake'] = 0.0
        R['accel'] = 0.70
        R['clutch'] = 0.0
        # In reverse gear, counter-steer relative to track position to rotate car back towards center
        R['steer'] = clip(-track_pos * 0.60 + angle * 0.40, -0.70, 0.70)
        state['prev_track_pos'] = track_pos
        if state['recovery_timer'] == 0:
            state['stuck_counter'] = 0
            state['gear_cooldown'] = 15
            R['gear'] = 1
            state['cmd_gear'] = 1
            R['accel'] = 0.30
        return

    # Stuck condition detection: near zero longitudinal velocity while off-center or facing barrier
    is_immobile = abs(speed_x) < 4.0
    is_off_center = abs(track_pos) > 0.45 or abs(angle) > 0.55
    is_front_blocked = min(track[8:11]) < 4.0

    if is_immobile and (is_off_center or is_front_blocked):
        state['stuck_counter'] += 1
    else:
        state['stuck_counter'] = max(0, state['stuck_counter'] - 1)

    # Reset stuck counter during initial countdown and standing launch
    if state['step'] < 60:
        state['stuck_counter'] = 0

    # Trigger 35-step (~0.7 sec) reverse escape maneuver if stuck for > 25 steps (~0.5s)
    if state['stuck_counter'] > 25:
        state['recovery_timer'] = 35
        state['stuck_counter'] = 0
        R['gear'] = -1
        state['cmd_gear'] = -1
        R['brake'] = 0.0
        R['accel'] = 0.70
        R['steer'] = clip(-track_pos * 0.60, -0.70, 0.70)
        state['prev_track_pos'] = track_pos
        return

    # -------------------------------------------------------------
    # 3. PREDICTIVE SPEED SCHEDULING (CORKSCREW TUNED)
    # -------------------------------------------------------------
    # Wide-Cone Radar Fusion (Sectors 6 through 12: approx -15 to +15 degrees)
    # Detects collapsing corner walls that narrow forward radar misses
    center_radar = min(track[8:11])
    wide_radar = min(track[6:13])
    front_radar_dist = min(center_radar, wide_radar * 1.15)

    # Uphill Incline Compensation:
    # When climbing (speed_z > 0.2 m/s), road slope artificially compresses radar rays.
    # Scale radar horizon by 18% to prevent false throttle lift on steep climbs.
    if speed_z > 0.2:
        front_radar_dist *= 1.18

    # Dynamic Speed Mapping
    if front_radar_dist >= 105.0:
        target_speed = 250.0
    elif front_radar_dist >= 62.0:
        target_speed = (165.0 + ((front_radar_dist - 62.0) / 43.0) * 70.0) * PACE
    elif front_radar_dist >= 38.0:
        target_speed = (110.0 + ((front_radar_dist - 38.0) / 24.0) * 60.0) * PACE
    else:
        floor = apex_floor(dist_from_start)
        target_speed = max(floor, floor + ((front_radar_dist - 12.0) / 26.0) * 20.0)

    # REFINE_V12: bounded speed-profile assist (raise-only, see PROFILE_* constants).
    profile_kmh = 0.0
    if PROFILE_ENABLED and dist_from_start < 3600.0:
        profile_raw = profile_speed_kmh(dist_from_start)
        on_line = (abs(track_pos) <= PROFILE_LINE_MAX_TRACKPOS and abs(angle) <= PROFILE_LINE_MAX_ANGLE
                   and abs(speed_y) <= PROFILE_LINE_MAX_SPEED_Y)
        wall_clear = center_radar >= (speed_x / 3.6) * PROFILE_RADAR_GUARD_S
        if profile_raw is not None and on_line and wall_clear:
            profile_kmh = min(profile_raw, target_speed + PROFILE_MAX_BOOST_KMH)
            target_speed = max(target_speed, profile_kmh)

    # Turn 8 position-gated plan (replaces v3's speed_z plunge clamp inside the corridor)
    target_speed, t8_phase = update_turn8_state(
        state, dist_from_start, speed_x, speed_z,
        angle, track_pos, state.get('last_steer', 0.0), target_speed
    )
    state['t8_phase'] = t8_phase
    t8_zone_active = t8_phase in ('T8_FORCE', 'T8_WAIT', 'T8_RAMP')

    # Hairpin approach: speed cap falls linearly to HAIRPIN_ENTRY_KMH at the corner entry.
    hairpin_zone_active = HAIRPIN_ZONE_START_DFS <= dist_from_start < HAIRPIN_ZONE_END_DFS
    if hairpin_zone_active:
        hairpin_cap = HAIRPIN_ENTRY_KMH + max(0.0, HAIRPIN_CORNER_DFS - dist_from_start) * HAIRPIN_SLOPE_KMH_PER_M
        target_speed = min(target_speed, hairpin_cap)

    # Right-hand bend approach (downhill): same idea, keeps the entry speed where refine_v3 had it.
    bend0_zone_active = BEND0_ZONE_START_DFS <= dist_from_start < BEND0_ZONE_END_DFS
    if bend0_zone_active:
        bend0_cap = BEND0_ENTRY_KMH + max(0.0, BEND0_CORNER_DFS - dist_from_start) * BEND0_SLOPE_KMH_PER_M
        target_speed = min(target_speed, bend0_cap)

    bend2_zone_active = BEND2_ZONE_START_DFS <= dist_from_start < BEND2_ZONE_END_DFS
    if bend2_zone_active:
        bend2_cap = BEND2_ENTRY_KMH + max(0.0, BEND2_CORNER_DFS - dist_from_start) * BEND2_SLOPE_KMH_PER_M
        target_speed = min(target_speed, bend2_cap)

    # Speed-Gated Cornering Yaw Derating:
    if abs(angle) > 0.22 and speed_x > 85.0:
        target_speed *= max(0.74, 1.0 - (abs(angle) - 0.22) * 0.38)

    target_speed = clip(target_speed, 54.0, 250.0)

    # FINISH LINE SPRINT: Corkscrew is 3608.45m long. 
    # Disable all cornering slow-downs in the final 125 meters to cross the line at max velocity.
    if dist_from_start > FINISH_SPRINT_DFS:
        target_speed = 300.0

    # -------------------------------------------------------------
    # 4. LATERAL STABILIZATION (PROPORTIONAL-DERIVATIVE STEERING)
    # -------------------------------------------------------------
    # Calculate position derivative to damp oscillations across centerline
    delta_track_pos = track_pos - state['prev_track_pos']
    state['prev_track_pos'] = track_pos

    # Dynamic Steering Law with Low-Speed Hairpin Authority
    # refine_v7: position gain scheduled with speed (see KP_* constants); the Turn 8 crawl keeps 0.36.
    if t8_phase in ('T8_FORCE', 'T8_WAIT'):
        kp = KP_BASE
    else:
        kp_blend = clip((KP_BASE_ABOVE_KMH - speed_x) / (KP_BASE_ABOVE_KMH - KP_FULL_BELOW_KMH), 0.0, 1.0)
        kp = KP_BASE + (KP_LOW_SPEED - KP_BASE) * kp_blend
    kd = 3.20
    k_angle = 0.85

    steer_raw = (angle * k_angle) - (track_pos * kp + delta_track_pos * kd)

    # High-speed stabilization vs low-speed turn-in agility
    if speed_x > 60.0:
        speed_stabilizer = max(1.0, 1.0 + (speed_x - 35.0) * 0.016)
        R['steer'] = clip(steer_raw / speed_stabilizer, -1.0, 1.0)
    else:
        # Full, crisp steering authority for hairpins below 60 km/h
        R['steer'] = clip(steer_raw * 1.15, -1.0, 1.0)

    state['last_steer'] = R['steer']

    # -------------------------------------------------------------
    # 5. TRACTION CONTROL (TCS) & ANTI-LOCK BRAKING SYSTEM (ABS)
    # -------------------------------------------------------------
    # Longitudinal Velocity Control
    if speed_x > target_speed:
        # Deceleration Phase
        speed_overshoot = speed_x - target_speed
        R['accel'] = 0.0

        # Downhill Minimal Braking Strategy
        # Inside the Turn-8 corridor gravity coasting is OFF: standard firm braking is used.
        if speed_z < -0.2 and front_radar_dist > 35.0 and not t8_zone_active and not hairpin_zone_active and not bend2_zone_active and not bend0_zone_active:
            if speed_overshoot < 25.0:
                raw_brake = 0.0  # Pure gravity coasting for up to 25 km/h overshoot
            else:
                # Apply only 30% of normal brake pressure to prevent aggressive slowing
                raw_brake = clip(speed_overshoot / BRAKE_GAIN_DIV, 0.0, 1.0) * 0.30
        else:
            # Standard Firm ABS deceleration for flat/uphill or immediate hairpins
            raw_brake = clip(speed_overshoot / BRAKE_GAIN_DIV, 0.0, 1.0)
            
            expected_front_spin = (speed_x / 3.6) / 0.33
            actual_front_spin = (wheel_spin[0] + wheel_spin[1]) / 2.0
            if speed_x > 25.0 and actual_front_spin < expected_front_spin * 0.65:
                raw_brake *= 0.40

        max_cornering_brake = max(0.35, 1.0 - abs(R['steer']) * 0.65)
        for zone_lo, zone_hi, zone_max_brake in BRAKE_SOFT_ZONES:
            if zone_lo <= dist_from_start < zone_hi:
                max_cornering_brake = min(max_cornering_brake, zone_max_brake)
        R['brake'] = clip(min(raw_brake, max_cornering_brake), 0.0, 1.0)
        
        # Absolute Brake Override for Finish Line Sprint
        if dist_from_start > FINISH_SPRINT_DFS:
            R['brake'] = 0.0

    else:
        # Acceleration Phase
        R['brake'] = 0.0
        speed_defect = target_speed - speed_x
        raw_accel = 1.0 if speed_defect > 5.0 else clip(speed_defect / 5.0, 0.40, 1.0)
        
        if speed_x < 15.0:
            raw_accel = 1.0

        # Traction Control System (TCS)
        rear_spin = wheel_spin[2] + wheel_spin[3]
        front_spin = wheel_spin[0] + wheel_spin[1]
        spin_diff = rear_spin - front_spin

        # EXPLOSIVE LAUNCH: Disable TCS throttling below 30 km/h to allow launch wheelspin
        if spin_diff > 3.0 and speed_x >= 30.0:
            tcs_reduction = clip((spin_diff - 3.0) * 0.075, 0.0, 0.70)
            raw_accel = max(0.20, raw_accel - tcs_reduction)

        R['accel'] = clip(raw_accel, 0.0, 1.0)

    # -------------------------------------------------------------
    # 6. SEQUENTIAL TRANSMISSION LOGIC
    # -------------------------------------------------------------
    # The shift logic runs on the gear we last COMMANDED, not on the gear sensor. The sensor lags a
    # shift by a few steps; rebuilding the gear from it every step re-sent the old gear during the
    # cooldown, cancelled each upshift and left the car hunting 3<->4 (or 1<->2) at the rev limiter
    # (see gravity_refine_log_20261013_234612.csv). The sensor is only used to resync after a
    # sustained disagreement (e.g. a server-side restart).
    if current_gear == state['cmd_gear']:
        state['gear_mismatch'] = 0
    else:
        state['gear_mismatch'] += 1
        if state['gear_mismatch'] > 30:
            state['cmd_gear'] = current_gear
            state['gear_mismatch'] = 0
    gear = state['cmd_gear']
    if gear <= 0:
        gear = 1

    # Dynamic Transmission
    gear_min_speeds = GEAR_MIN_SPEEDS

    upshift_rpm = UPSHIFT_RPM_UPHILL if speed_z > 0.2 else UPSHIFT_RPM_FLAT

    # Force Gear 1 kickdown below 60 km/h in tight hairpins (relaxed in T8_RAMP)
    kickdown_threshold = T8_KICKDOWN_SPEED_RELEASE if state.get('t8_phase') == 'T8_RAMP' else 60.0

    if state['gear_cooldown'] == 0:
        # Never upshift into a gear that the downshift rules below would undo straight away
        # (below the kickdown speed, or below the next gear's minimum speed).
        can_upshift = (gear < 6 and speed_x >= kickdown_threshold and
                       speed_x >= gear_min_speeds.get(gear + 1, 0.0))
        if rpm > upshift_rpm and can_upshift:
            gear += 1
            state['gear_cooldown'] = 8
        elif gear > 1:
            if speed_x < kickdown_threshold and gear > 1:
                gear = 1
                state['gear_cooldown'] = 10
            elif rpm < 4000 or speed_x < gear_min_speeds.get(gear, 0.0):
                gear -= 1
                state['gear_cooldown'] = 8

    state['cmd_gear'] = gear
    R['gear'] = gear
    R['clutch'] = 0.0
    R['meta'] = 0

    # -------------------------------------------------------------
    # 7. TELEMETRY LOGGING
    # -------------------------------------------------------------
    if LOG_TELEMETRY and (state['step'] % 5 == 0):
        try:
            log_filename = state.get('log_file', 'gravity_refine_log.csv')
            write_header = not os.path.exists(log_filename)
            with open(log_filename, 'a') as f:
                if write_header:
                    f.write("step,distFromStart,distRaced,speedX,speedZ,angle,trackPos,steer,accel,brake,gear,rpm,min(track[8:11]),target_speed,t8_phase,t8_cap,curLapTime,sensor_gear,profile_kmh\n")
                f.write(
                    f"{state['step']},"
                    f"{dist_from_start:.2f},"
                    f"{dist_raced:.2f},"
                    f"{speed_x:.2f},"
                    f"{speed_z:.2f},"
                    f"{angle:.4f},"
                    f"{track_pos:.4f},"
                    f"{R['steer']:.4f},"
                    f"{R['accel']:.4f},"
                    f"{R['brake']:.4f},"
                    f"{R['gear']},"
                    f"{rpm:.1f},"
                    f"{min(track[8:11]):.2f},"
                    f"{target_speed:.2f},"
                    f"{state.get('t8_phase', 'V3')},"
                    f"{state.get('t8_cap', 54.0):.2f},"
                    f"{float(S.get('curLapTime', 0.0)):.2f},"
                    f"{current_gear},"
                    f"{profile_kmh:.1f}\n"
                )
        except Exception:
            pass

# ================ MAIN ================
if __name__ == "__main__":
    #### DO NOT HARCODE THE PORT
    C= Client()
    for step in range(C.maxSteps,0,-1):
        C.get_servers_input()
        drive_example(C)
        C.respond_to_server()
    C.shutdown()
