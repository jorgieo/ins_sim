"""Packaged default configuration files.

ins_sim ships one mission YAML and one IMU-spec YAML. The GUI copies them
into its user folders on first launch; scripts can load them directly
through these paths.
"""

from importlib import resources
from pathlib import Path


def default_imu_spec_path() -> Path:
    """Returns the path of the packaged navigation-grade IMU spec.

    Returns:
        pathlib.Path: Path to ``imu_spec.yaml`` inside the installed
            package.

    Examples:
        >>> from ins_sim.config import default_imu_spec_path
        >>> default_imu_spec_path().name
        'imu_spec.yaml'
    """
    return resources.files("ins_sim.config") / "imu_spec.yaml"


def default_trajectory_path() -> Path:
    """Returns the path of the packaged mission (a departure from BQN airport).

    Returns:
        pathlib.Path: Path to ``bqn_departure.yaml`` inside the installed
            package.

    Examples:
        >>> from ins_sim.config import default_trajectory_path
        >>> default_trajectory_path().name
        'bqn_departure.yaml'
    """
    return resources.files("ins_sim.config") / "bqn_departure.yaml"
