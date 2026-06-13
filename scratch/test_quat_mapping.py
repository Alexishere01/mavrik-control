import numpy as np
import math

def quat_to_euler(e0, ex, ey, ez):
    sinp = max(-1.0, min(1.0, 2.0 * (e0 * ey - ez * ex)))
    return (math.atan2(2.0 * (e0 * ex + ey * ez), 1.0 - 2.0 * (ex * ex + ey * ey)),
            math.asin(sinp),
            math.atan2(2.0 * (e0 * ez + ex * ey), 1.0 - 2.0 * (ey * ey + ez * ez)))

# MAVRIK system: +x=Fwd, +y=Right, +z=Up.
# This is LHS because x x y = -z.
# RHS system (NED body): +x=Fwd, +y=Right, +z=Down.
# So transform matrix is diag(1, 1, -1).

# Let's test a roll right of 10 degrees in LHS:
# In LHS (+x=Fwd, +y=Right, +z=Up), roll right (tilted right, right wing down)
# is a rotation about +x that rotates +y (Right) down (towards -z).
# By right-hand rule, roll right is negative rotation about +x.
# Wait, let's check MAVRIK's native behavior:
# Let's see what angles Option B (e0, -ex, ey, -ez) vs others produce.
phi = math.radians(10.0) # 10 deg
e0 = math.cos(phi/2)
ex = math.sin(phi/2) # positive for roll right in some convention?
# Let's test all combinations of signs for e0, ex, ey, ez
print("For LHS roll (ex=sin(phi/2)):")
for signs in [
    (1, 1, 1, 1),
    (1, -1, 1, 1),
    (1, 1, -1, 1),
    (1, 1, 1, -1),
    (1, -1, -1, 1),
    (1, -1, 1, -1),
    (1, 1, -1, -1),
    (1, -1, -1, -1)
]:
    q = (e0, signs[1]*ex, 0.0, 0.0)
    r, p, y = quat_to_euler(*q)
    print(f"  signs={signs} -> roll={math.degrees(r):.2f}° pitch={math.degrees(p):.2f}°")

print("\nFor LHS pitch (ey=sin(theta/2)):")
theta = math.radians(10.0)
e0 = math.cos(theta/2)
ey = math.sin(theta/2)
for signs in [
    (1, 1, 1, 1),
    (1, -1, 1, 1),
    (1, 1, -1, 1),
    (1, 1, 1, -1),
    (1, -1, -1, 1),
    (1, -1, 1, -1),
    (1, 1, -1, -1),
    (1, -1, -1, -1)
]:
    q = (e0, 0.0, signs[2]*ey, 0.0)
    r, p, y = quat_to_euler(*q)
    print(f"  signs={signs} -> roll={math.degrees(r):.2f}° pitch={math.degrees(p):.2f}°")
