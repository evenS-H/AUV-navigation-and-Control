from . import PID_params as P
from .PID_Core import compute_error, wrap_to_pi, PIDController


"""
PID runner for yaw and pitch axes.

Inputs each step:
  yaw_ref, yaw_meas    [rad]
  pitch_ref, pitch_meas [rad]

Outputs each step:
  u_yaw, u_pitch  (actuator commands, rad)
"""

_yaw_pid   = PIDController(P.YAW)
_pitch_pid = PIDController(P.PITCH)


def step(yaw_ref: float, yaw_meas: float, pitch_ref: float, pitch_meas: float):
    """
    Compute PID outputs for yaw and pitch.

    Yaw error is wrapped to [-pi, pi].
    Pitch error is not wrapped (bounded by physical limits).

    Returns:
        u_yaw   (float): yaw actuator command [rad]
        u_pitch (float): pitch actuator command [rad]
    """
    e_yaw   = wrap_to_pi(compute_error(yaw_ref, yaw_meas))
    e_pitch = compute_error(pitch_ref, pitch_meas)

    u_yaw   = _yaw_pid.update(e_yaw)
    u_pitch = _pitch_pid.update(e_pitch)

    return u_yaw, u_pitch
