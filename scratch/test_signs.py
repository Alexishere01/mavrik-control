import math
from typing import Tuple

def quat_to_euler(e0, ex, ey, ez) -> Tuple[float, float, float]:
    phi = math.atan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex*ex + ey*ey))
    theta = math.asin(max(-1.0, min(1.0, 2 * (e0 * ey - ez * ex))))
    psi = math.atan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey*ey + ez*ez))
    return phi, theta, psi

# In MAVRIK LHS, let's pitch the nose UP by 10 degrees.
# MAVRIK LHS positive pitch is nose DOWN, so pitch UP is -10 deg.
# So raw MAVRIK q_mav should represent -10 deg pitch around y Left.
# q_mav = (cos(-5 deg), 0, sin(-5 deg), 0)
theta_mav = -10 * math.pi / 180
q_mav = (math.cos(theta_mav/2), 0.0, math.sin(theta_mav/2), 0.0)

# Standard RHS expects pitch UP 10 degrees to be +10 deg pitch.
# Let's check what Euler pitch we get with q_ned = (q_mav[0], q_mav[1], q_mav[2], -q_mav[3]):
q_ned_no_neg_ey = (q_mav[0], q_mav[1], q_mav[2], -q_mav[3])
r1, p1, y1 = quat_to_euler(*q_ned_no_neg_ey)

# Let's check with q_ned = (q_mav[0], q_mav[1], -q_mav[2], -q_mav[3]):
q_ned_neg_ey = (q_mav[0], q_mav[1], -q_mav[2], -q_mav[3])
r2, p2, y2 = quat_to_euler(*q_ned_neg_ey)

print(f"With positive ey: roll={math.degrees(r1):.2f} pitch={math.degrees(p1):.2f} yaw={math.degrees(y1):.2f}")
print(f"With negated ey : roll={math.degrees(r2):.2f} pitch={math.degrees(p2):.2f} yaw={math.degrees(y2):.2f}")
print(f"Expected RHS   : roll=0.00 pitch=10.00 yaw=0.00")

# Let's test roll RIGHT by 10 degrees.
# MAVRIK LHS positive roll is roll RIGHT, so +10 deg.
phi_mav = 10 * math.pi / 180
q_mav_roll = (math.cos(phi_mav/2), math.sin(phi_mav/2), 0.0, 0.0)

q_ned_roll = (q_mav_roll[0], q_mav_roll[1], -q_mav_roll[2], -q_mav_roll[3])
r_roll, p_roll, y_roll = quat_to_euler(*q_ned_roll)
print(f"Roll RIGHT 10deg: roll={math.degrees(r_roll):.2f} pitch={math.degrees(p_roll):.2f} yaw={math.degrees(y_roll):.2f}")
print(f"Expected RHS   : roll=10.00 pitch=0.00 yaw=0.00")

