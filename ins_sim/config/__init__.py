from importlib import resources
from pathlib import Path


def default_imu_spec_path() -> Path:
    return resources.files("ins_sim.config") / "imu_spec.yaml"


def default_trajectory_path() -> Path:
    return resources.files("ins_sim.config") / "bqn_departure.yaml"
