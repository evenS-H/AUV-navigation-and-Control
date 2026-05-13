import math

import numpy as np


def compute_error(ref: float, meas: float) -> float:
    return ref - meas 


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PIDController:
    """
    PID controller with back-calculation anti-windup.

    The params object must provide:
      DT, TT, U_MIN, U_MAX
      get_gains(error, edot) -> (Kp, Ki, Kd)
    """

    def __init__(self, params):
        self.P = params
        self.ui = 0.0
        self.prev_error = 0.0

    def update(self, error: float) -> float:
        """
        Performs one PID step with back-calculation anti-windup.
        Returns the saturated control output.
        """
        P = self.P

        edot = (error - self.prev_error) / P.DT

        Kp, Ki, Kd = P.get_gains(error, edot)

        up = Kp * error
        ud = Kd * edot

        u = up + self.ui + ud

        # Saturate
        ur = np.clip(u, P.U_MIN, P.U_MAX)

        # Back-calculation anti-windup
        ep = ur - u
        self.ui = self.ui + (Ki * error + (1.0 / P.TT) * ep) * P.DT

        # Final output with saturation
        u_star = np.clip(up + self.ui + ud, P.U_MIN, P.U_MAX)

        self.prev_error = error
        return float(u_star)
