"""
CT vs J Curve Fitting for T-Motor VZ32x12 (32x12 prop)
=======================================================
Use this script to:
  1. Fit your measured thrust data to CT = f(J) polynomial
  2. Fit power data to CPb = f(J) polynomial
  3. Export the coefficients directly into Team1_VTOL.json format

How to use:
-----------
  1. Fill in your measured data arrays below (from thrust stand tests)
  2. Run:  python3 fit_thrust_curve.py
  3. Copy the printed JSON snippet into Team1_VTOL.json under each motor's
     "aerodynamics" -> "polynomials" -> "CT" and "CPb" sections

Advance ratio J:
    J = V / (n * D)
    where:
        V = airspeed [ft/s or m/s - be consistent]
        n = rotational speed [rev/s]   (RPM / 60)
        D = prop diameter [ft or m]    (32 in = 2.667 ft = 0.8128 m)

Thrust coefficient CT:
    T = CT * rho * n^2 * D^4
    CT = T / (rho * n^2 * D^4)

Power coefficient CPb:
    P = CPb * rho * n^3 * D^5
    CPb = P / (rho * n^3 * D^5)

Standard sea-level density:
    rho = 0.002377 slug/ft^3  (US customary, consistent with MAVRIK default units)
    rho = 1.225 kg/m^3        (SI)
"""

import numpy as np
from numpy.polynomial import polynomial as P
import json

# ============================================================
# 1. FILL IN YOUR TEST DATA HERE
#    Each row: [RPM, airspeed_m/s, thrust_N, power_W]
#    Static test = airspeed 0. Sweep RPM and/or windspeed.
# ============================================================
test_data = [
    # RPM     V[m/s]   T[N]     P[W]
    # ---- REPLACE WITH YOUR MEASURED VALUES ----
    # Example placeholder data (linear approximation):
    [1500,   0.0,    441.45,  8000],   # full throttle static
    [1200,   5.0,    320.0,   5500],
    [1000,  10.0,    220.0,   3800],
    [ 800,  15.0,    130.0,   2200],
    [ 600,  20.0,     60.0,   1000],
    [ 400,  25.0,     10.0,    300],
]

# Prop geometry
D_m  = 0.8128       # diameter in meters  (32 in = 0.8128 m)
D_ft = 2.6667       # diameter in feet

rho_si = 1.225      # kg/m^3 at sea level
rho_us = 0.002377   # slug/ft^3 at sea level

poly_order = 2      # 2nd order: CT = a0 + a1*J + a2*J^2

# ============================================================
# 2. COMPUTE CT, CPb, J
# ============================================================
data = np.array(test_data)
RPM = data[:, 0]
V   = data[:, 1]   # m/s
T   = data[:, 2]   # N
Pw  = data[:, 3]   # W

n   = RPM / 60.0   # rev/s

J   = V / (n * D_m)
CT  = T  / (rho_si * n**2 * D_m**4)
CPb = Pw / (rho_si * n**3 * D_m**5)

print("--- Computed Aerodynamic Coefficients ---")
print(f"{'J':>8}  {'CT':>10}  {'CPb':>10}")
for j, ct, cp in zip(J, CT, CPb):
    print(f"{j:8.4f}  {ct:10.6f}  {cp:10.6f}")

# ============================================================
# 3. POLYNOMIAL FIT
# ============================================================
# Filter out J < 0 or nan
mask = np.isfinite(J) & (J >= 0)
J_fit   = J[mask]
CT_fit  = CT[mask]
CPb_fit = CPb[mask]

ct_coeffs  = np.polyfit(J_fit, CT_fit,  poly_order)   # highest power first
cpb_coeffs = np.polyfit(J_fit, CPb_fit, poly_order)

print("\n--- Polynomial Fit (CT vs J) ---")
print(f"  CT  = {ct_coeffs[2]:.6f}  +  {ct_coeffs[1]:.6f}*J  +  {ct_coeffs[0]:.6f}*J^2")
print(f"  CPb = {cpb_coeffs[2]:.6f}  +  {cpb_coeffs[1]:.6f}*J  +  {cpb_coeffs[0]:.6f}*J^2")

# ============================================================
# 4. OUTPUT JSON SNIPPET
# ============================================================
snippet = {
    "aerodynamics": {
        "polynomials": {
            "CT": {
                "0":    round(float(ct_coeffs[2]), 6),
                "J":    round(float(ct_coeffs[1]), 6),
                "J^2":  round(float(ct_coeffs[0]), 6)
            },
            "CPb": {
                "0":    round(float(cpb_coeffs[2]), 6),
                "J":    round(float(cpb_coeffs[1]), 6),
                "J^2":  round(float(cpb_coeffs[0]), 6)
            },
            "CNa": {"0": 0.3},
            "Cna": {"0": -0.007}
        }
    }
}

print("\n--- JSON snippet to paste into Team1_VTOL.json (all 4 motors) ---")
print(json.dumps(snippet, indent=4))

# ============================================================
# 5. HOVER CHECK
# ============================================================
# At hover, J=0, CT = CT[0]. Estimate hover RPM.
# Each motor lifts (MTOW / 4) at hover unloaded
MTOW_kg  = 21.723218   # typical operating weight unloaded
T_hover_N = (MTOW_kg * 9.81) / 4
CT0 = float(ct_coeffs[2])

# T = CT * rho * n^2 * D^4  =>  n = sqrt(T / (CT * rho * D^4))
if CT0 > 0:
    n_hover = np.sqrt(T_hover_N / (CT0 * rho_si * D_m**4))
    RPM_hover = n_hover * 60
    throttle_hover = RPM_hover / (150 * 51.8)   # KV * V_batt
    print(f"\n--- Hover Check (unloaded, {MTOW_kg:.1f} kg) ---")
    print(f"  Required thrust per motor: {T_hover_N:.1f} N")
    print(f"  Hover RPM estimate:        {RPM_hover:.0f}")
    print(f"  Hover throttle estimate:   {throttle_hover:.3f}  (target ~0.3-0.5)")
