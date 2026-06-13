# Walkthrough: MAVRIK Tailless Transition Stabilization

This document summarizes the final "Winning Configuration" for the MAVRIK VTOL. Use these settings when re-applying the transition logic to a clean directory.

## 1. Physical Configuration (Team1_VTOL.json)
We converted the simulation to a **Tailless Flying Wing** to match the physical drone prototype.
- **Removed Tail**: Deleted `h_stab` and `v_stab` components.
- **Center of Gravity**: Moved fuselage to `x=1.6`. This puts the CG slightly forward of the wing midpoint for stability.
- **Wing Properties**: Set `Cm0 = 0.0` for both wing halves to neutralize the pitching moment.
- **Effectors**: Defined `elevonLeft` and `elevonRight` as the primary control surfaces.

## 2. Control Bridge (mavrik_ardupilot_bridge.py)
The bridge was overhauled to sync ArduPilot's logic with the new physics.
- **Motor Mapping (The "Smoking Gun")**:
  - AP CH1 (FR) -> MAVRIK **Aft-Right**
  - AP CH2 (RL) -> MAVRIK **Fwd-Left**
  - AP CH3 (FL) -> MAVRIK **Aft-Left**
  - AP CH4 (RR) -> MAVRIK **Fwd-Right**
  - *Rationale*: This syncs ArduPilot's internal "Rear" commands with MAVRIK's "Front" motors, fixing the pitch-reversal bug.
- **Elevon Mixing**:
  - `elevon_l = elevator + aileron`
  - `elevon_r = elevator - aileron`
- **Packet Format**: Updated `MAVRIK_CONTROL_FMT` to `"<8f"` (removed the 9th rudder float).
- **Polarity**: Reversed Elevator (`* -15.0`) and Rudder (`* -15.0`) to match MAVRIK's "Negative = UP/RIGHT" convention.

## 3. Autopilot Parameters (mavrik.parm)
- **PID Gains**: Reduced `Q_A_RAT_PIT_P` and `Q_A_RAT_RLL_P` to `0.015`. Added `Q_A_RAT_PIT_D 0.005` for damping.
- **Transition Safety**:
  - `Q_TILT_MAX 45`: Temporarily limited to 45 degrees for stability testing.
  - `Q_TRANSITION_MS 2000`: Fast 2-second transition to punch through the low-speed zone.
- **Thrust Limits**: `Q_M_SPIN_MAX 0.75` and `THR_MAX 40` for adequate headroom.

## 4. How to Test
1. Run `./start_terminals.sh`.
2. Take off in `QHOVER`.
3. Accelerate to 40 ft/s.
4. Switch to `FBWA`.
5. Observe the nose: It should track straight and level with the new elevon logic.
