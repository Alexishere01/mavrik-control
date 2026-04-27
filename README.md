# Team 1 — MAVRIK Simulation Package
**Aircraft:** Hybrid Tilt-Rotor VTOL (2 tilt + 2 fixed VTOL motors)
**Motor:** T-Motor VL1040 KV150 + VZ32*12 prop
**Target:** Transition to forward flight / PID tuning

## Files
| File | Purpose |
|---|---|
| `input_Team1.json` | Main MAVRIK input — run this |
| `Team1_VTOL.json` | Vehicle definition (geometry, motors, aero, payload) |
| `connections.json` | Data logging config |
| `fit_thrust_curve.py` | CT vs J curve fitter — use when you have wind tunnel data |

## How to Run
1. Copy all 4 files into your `zWindowsDeployment` folder alongside `mavrik.exe` and `license.txt`
2. Open a terminal in that folder
3. Run: `mavrik.exe input_Team1.json`

## Payload Configs
In `Team1_VTOL.json`, find the `payload_slot` component and change:
- `"weight[kg]": 0` → no payload
- `"weight[kg]": 49.895` + `"location[m]": [0.274744, 0, -0.15731]` → 110 lbs
- `"weight[kg]": 74.843` + `"location[m]": [0.274744, 0, -0.123691]` → 165 lbs
- `"weight[kg]": 99.79`  + `"location[m]": [0.274744, 0, -0.123334]` → 220 lbs

## Motor Parameters (from VL1040 spec, 60V bench test)
- CT0 = 0.087989 (validated: predicts exactly 45 kg at 5812 RPM)
- 14S LiPo discharge curve embedded (42V empty → 58.8V full)
- Hover throttle (unloaded): ~23% per motor

## State-Space Output (for PID tuning)
MAVRIK will generate `Team1_VTOL_SS_A.csv` and `Team1_VTOL_SS_B.csv`.
Use these A/B matrices in MATLAB or Python `control` library to design PID gains.

## What Still Needs Real Data
- **CT vs J at forward airspeed** — currently estimated from static bench data.
  Run `fit_thrust_curve.py` with wind tunnel / car test measurements to update.
- **Inertia with payload** — CAD values are for unloaded config.
