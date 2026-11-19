"""The packaged IMU spec: datasheet values, and the model parameters derived from them."""

import numpy as np

from ins_sim.config import default_imu_spec_path
from ins_sim.sensors.imu import GM_ALLAN_PEAK, load_imu_spec

spec = load_imu_spec(str(default_imu_spec_path()))   # converted to SI on loading
tau_i = 0.1                                          # IMU output interval [s]
DEG_PER_HOUR = np.deg2rad(1.0) / 3600.0              # 1 °/h in rad/s
MICRO_G = 9.80665e-6                                 # 1 µg in m/s²


def show(label: str, value: float, unit: str) -> None:
    print(f"  {label:<42s}{value:9.4f} {unit}")


print("Gyro")
show("bias instability B (datasheet)", spec.gyro_bias_instability / DEG_PER_HOUR, "°/h")
show(f"Gauss-Markov spread, B / {GM_ALLAN_PEAK:.4f}", spec.gyro_gm_std / DEG_PER_HOUR, "°/h")
show("white noise per sample, ARW / sqrt(tau_i)", spec.gyro_arw / np.sqrt(tau_i) / DEG_PER_HOUR, "°/h")
print("Accelerometer")
show("bias instability B (datasheet)", spec.accel_bias_instability / MICRO_G, "µg")
show(f"Gauss-Markov spread, B / {GM_ALLAN_PEAK:.4f}", spec.accel_gm_std / MICRO_G, "µg")
show("white noise per sample, VRW / sqrt(tau_i)", spec.accel_vrw / np.sqrt(tau_i) / MICRO_G, "µg")
print("Initial alignment")
show("tilt spread", np.rad2deg(spec.align_tilt_std), "°")
show("heading spread", np.rad2deg(spec.align_heading_std), "°")
