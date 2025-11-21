import numpy as np

fovy_rad = np.deg2rad(60)
height = 480
width = 640

# Calculate focal length in pixels
fy = (height / 2.0) / np.tan(fovy_rad / 2.0)
fx = fy  # Assuming square pixels (usually true for simulation)

# Principal point (usually image center)
cx = width / 2.0
cy = height / 2.0

print(f"Camera.fx: {fx:.6f}")
print(f"Camera.fy: {fy:.6f}")
print(f"Camera.cx: {cx:.6f}")
print(f"Camera.cy: {cy:.6f}")