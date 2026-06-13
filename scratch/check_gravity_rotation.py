import random
import math

def q_to_matrix(q):
    e0, ex, ey, ez = q
    return [
        [1.0 - 2.0*(ey**2 + ez**2), 2.0*(ex*ey - e0*ez),     2.0*(ex*ez + e0*ey)],
        [2.0*(ex*ey + e0*ez),     1.0 - 2.0*(ex**2 + ez**2), 2.0*(ey*ez - e0*ex)],
        [2.0*(ex*ez - e0*ey),     2.0*(ey*ez + e0*ex),     1.0 - 2.0*(ex**2 + ey**2)]
    ]

def mat_vec_mul(M, v):
    return (
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
    )

# Standard quat_to_euler
def quat_to_euler(e0, ex, ey, ez):
    sinp = max(-1.0, min(1.0, 2.0 * (e0 * ey - ez * ex)))
    return (math.atan2(2.0 * (e0 * ex + ey * ez), 1.0 - 2.0 * (ex * ex + ey * ey)),
            math.asin(sinp),
            math.atan2(2.0 * (e0 * ez + ex * ey), 1.0 - 2.0 * (ey * ey + ez * ez)))

# Let's test a simple 10 deg yaw right:
# MAVRIK LHS state: yaw right 10 deg -> ez < 0? Let's check both ez > 0 and ez < 0.
# In LHS, positive yaw is yaw left, so yaw right is ez < 0.
psi = math.radians(-10.0) # yaw right
e0 = math.cos(psi/2)
ex = 0.0
ey = 0.0
ez = math.sin(psi/2)

print("LHS state: yaw right 10 deg")
print(f"MAVRIK: e0={e0:.4f}, ex={ex:.4f}, ey={ey:.4f}, ez={ez:.4f}")

# Candidate 1: Option D: (e0, ex, ey, -ez)
q_optD = (e0, ex, ey, -ez)
r, p, y = quat_to_euler(*q_optD)
print(f"Option D Euler: roll={math.degrees(r):.2f}° pitch={math.degrees(p):.2f}° yaw={math.degrees(y):.2f}°")

# Candidate 2: Option F (or check_quat Option B): (e0, -ex, ey, -ez)
q_optF = (e0, -ex, ey, -ez)
r, p, y = quat_to_euler(*q_optF)
print(f"Option F Euler: roll={math.degrees(r):.2f}° pitch={math.degrees(p):.2f}° yaw={math.degrees(y):.2f}°")

# Now let's check gravity in body frame for pitch up:
# In Earth frame, gravity is (0, 0, 9.81) pointing down.
# For pitch up 10 deg, body x is pointing slightly up, so gravity should have a negative x component:
# g_body_x = -9.81 * sin(10 deg) = -1.70 m/s^2
# g_body_z = 9.81 * cos(10 deg) = 9.66 m/s^2
# Let's check what our earth_to_body rotation gives!
M_optD = q_to_matrix(q_optD)
M_T_optD = [[M_optD[j][i] for j in range(3)] for i in range(3)]
g_body_optD = mat_vec_mul(M_T_optD, (0, 0, 9.80665))
print(f"Option D g_body: {g_body_optD}")

M_optF = q_to_matrix(q_optF)
M_T_optF = [[M_optF[j][i] for j in range(3)] for i in range(3)]
g_body_optF = mat_vec_mul(M_T_optF, (0, 0, 9.80665))
print(f"Option F g_body: {g_body_optF}")
