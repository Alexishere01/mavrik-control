import random
import math

def q_mul(a, b):
    return (
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    )

def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])

def rotate(v, q):
    # q * v * q_conj
    qv = q_mul(q, (0, v[0], v[1], v[2]))
    qvq = q_mul(qv, q_conj(q))
    return (qvq[1], qvq[2], qvq[3])

# Generate a random LHS quaternion by rotating by roll, pitch, yaw in LHS
roll = random.uniform(-1, 1)
pitch = random.uniform(-1, 1)
yaw = random.uniform(-1, 1)

# In LHS:
# R_x(roll) * R_y(pitch) * R_z(yaw)
# Since LHS, let's just generate a random unit quaternion q_lhs:
q_lhs = [random.uniform(-1, 1) for _ in range(4)]
mag = math.sqrt(sum(x*x for x in q_lhs))
q_lhs = tuple(x/mag for x in q_lhs)

# Random body vector in LHS:
v_body_lhs = (random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))

# Rotate to Earth in LHS:
v_earth_lhs = rotate(v_body_lhs, q_lhs)

# Transform LHS body and earth vectors to RHS NED:
# RHS NED: x_rhs = x_lhs, y_rhs = y_lhs, z_rhs = -z_lhs
v_body_rhs = (v_body_lhs[0], v_body_lhs[1], -v_body_lhs[2])
v_earth_rhs = (v_earth_lhs[0], v_earth_lhs[1], -v_earth_lhs[2])

# Now let's try different RHS quaternion candidates to see which one rotates v_body_rhs to v_earth_rhs!
candidates = {
    "option A: (e0, ex, ey, ez)": (q_lhs[0], q_lhs[1], q_lhs[2], q_lhs[3]),
    "option B: (e0, -ex, -ey, ez)": (q_lhs[0], -q_lhs[1], -q_lhs[2], q_lhs[3]),
    "option C: (e0, ex, ey, -ez)": (q_lhs[0], q_lhs[1], q_lhs[2], -q_lhs[3]),
    "option D: (e0, -ex, ey, ez)": (q_lhs[0], -q_lhs[1], q_lhs[2], q_lhs[3]),
    "option E: (e0, ex, -ey, ez)": (q_lhs[0], q_lhs[1], -q_lhs[2], q_lhs[3]),
    "option F: (e0, -ex, ey, -ez)": (q_lhs[0], -q_lhs[1], q_lhs[2], -q_lhs[3]),
    "option G: (e0, ex, -ey, -ez)": (q_lhs[0], q_lhs[1], -q_lhs[2], -q_lhs[3]),
    "option H: (e0, -ex, -ey, -ez)": (q_lhs[0], -q_lhs[1], -q_lhs[2], -q_lhs[3]),
}

print("Checking rotation from RHS body to RHS earth:")
found = False
for name, q_rhs in candidates.items():
    v_earth_rhs_pred = rotate(v_body_rhs, q_rhs)
    error = math.sqrt(sum((a-b)**2 for a, b in zip(v_earth_rhs, v_earth_rhs_pred)))
    if error < 1e-7:
        print(f"  ✓ {name} matches perfectly!")
        found = True
    else:
        pass

if not found:
    print("  ✗ No candidate matched!")
