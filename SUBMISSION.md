To: ibmskillsbuild.emea@skillup.online  
Subject: AI Racing League Project Submission - Sriharshini Manda  

Hello Team,  

Please find my project submission for evaluation.  

Name: Sriharshini Manda  
Project Members: None  
GitHub Link: https://github.com/Sriharshini-Manda/ai-racing-final-sriharshini  
Video Demonstration: https://drive.google.com/file/d/1AnSqm8uMnyo1-LQUDmnEmm1W6EgikYjb/view?usp=sharing  

Project Description / Problem Statement:  
Situation: The Corkscrew circuit in TORCS combines long straights, a dozen bends of different severity and Turn 8, a blind crest with a roughly five-storey drop, and my first drivers either lost minutes to caution or damaged the car there.  
Task: Write a Python SCR client that completes one standing-start lap as fast as possible with zero car damage.  
Action: I improved the driver through a measured series of versions, each one a new file diagnosed from the screen recording and a per-step telemetry log, ending in snakeoil3_gravity_refine_v12.py.  
Result: The final driver completes the lap in 01:48:28 with 0 damage and a top speed of 239 km/h, down from 04:01:80 for my first safe baseline.  

Enhancements:  
1. Turn 8 position-gated plan: speed is decided only by distance along the lap (forced 57 km/h crawl through the S-bend from 2,380 m, a controlled approach and a ramp with an abort rule afterwards) instead of a latched sensor trigger. SCR speeds are in km/h, so the earlier trigger (speedZ below -1.9) fired on a 0.5 m/s blip, released the car about 2 km too early and crashed it into the wall at the Corkscrew. The plan removes that failure mode: Turn 8 is now taken at a peak lateral offset of 0.92 with zero damage, and the descent afterwards runs up to 150 km/h instead of a 54 km/h crawl.  
2. Soft centerline steering: the lane-keeping gain is scheduled with speed (0.60 up to 80 km/h, falling to 0.36 at 130 km/h). A proportional-only lane keeper settles at a constant offset in a bend (offset = steer needed x stabilizer / kp), so the car was riding the track edge in every tight bend although steering was below 30% of lock. A kinematic lane-keeping model reproduced the measured offsets and showed that the scheduled gain removes about 35% of the offset without ringing at speed. Measured peak offsets fell from 1.02 / 1.19 / 0.90 to 0.80 / 0.94 / 0.63, and the lap time fell by 5.06 s in one iteration.  
3. Safe speed-profile assist: a distance-indexed speed table lets the car brake later and accelerate earlier where the radar is too cautious, but it can only raise the target and is bounded by what the car has already driven (30 m look-ahead curvature, 15 m/s2 braking, a cap at the proven speed plus 40 km/h, only plus 8 km/h near corners, a radar guard, exclusion of the Turn 8 and downhill section, and on-line gating). An earlier unbounded version of the table sent the car off the road at three bends because the steering-derived curvature lagged the real corner entry. The bounded version enters the same bends at the proven speeds with the same lateral offsets and no damage.  

Use of IBM Bob( Free trial version)  
I used AI coding assistants, including Gemini and Claude Sonnet, alongside the IBM Bob free trial where credits allowed.  
Situation: I was a single developer iterating on a driver; the AI assistants had no simulator access and could only work from my screen recordings and telemetry logs.  
Task: Use the assistants for code refactoring, TORCS physics debugging, kinematic formulas and CSV telemetry analysis, while I stayed responsible for running and judging every version.  
Action: For refactoring, the assistants turned the controller into a position-gated Turn 8 state machine with table-driven zones, a gearbox that works on the commanded gear, and constants that keep their previous values in comments, always in a new file so the previous best was untouched. For TORCS physics debugging, they found from the logs that SCR speeds are in km/h, that the gear sensor lags a shift, that upshift thresholds below the next gear's minimum speed caused 116 gear flaps per lap, and that a legacy clamp acted as an accidental brake zone. For kinematic formulas, they derived the proportional-only lane-keeping offset, a lateral-acceleration estimate from steering, a lap-time model and torque-based shift points. For CSV telemetry, they added a per-step logger and used per-100 m comparisons and replays of logged states to find each loss. I ran every version in TORCS, checked the diffs, compiled each file, confirmed the previous version's checksum was unchanged, compared lap time and damage, and rejected ideas that did not hold up, including an integral steering term and the first speed table.  
Result: Lap time fell from 04:01:80 to 01:48:28 with 0 damage, every version is reproducible from its own file and log, and the failures are documented in the recordings.  

Thank you,  
Sriharshini Manda.  
