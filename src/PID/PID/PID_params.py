import math

# ============================================================
# Timing / actuator limits
# ============================================================
DT = 0.04
TT = 2.0

U_MIN_YAW   = -16 * math.pi / 180.0
U_MAX_YAW   =  16 * math.pi / 180.0

U_MIN_PITCH = -16 * math.pi / 180.0
U_MAX_PITCH =  16 * math.pi / 180.0

# ============================================================
# PID gains
# ============================================================
YAW_KP,   YAW_KI,   YAW_KD   = 0.1,  0.01, 0.01
PITCH_KP, PITCH_KI, PITCH_KD = 0.2,  0.05, 0.05


class AxisParams:
    """
    Parameter container for one control axis.

    get_gains() returns fixed (Kp, Ki, Kd) — no adaptive or fuzzy logic.
    """

    def __init__(self, DT, TT, U_MIN, U_MAX, Kp, Ki, Kd):
        self.DT    = DT
        self.TT    = TT
        self.U_MIN = U_MIN
        self.U_MAX = U_MAX
        self.Kp    = Kp
        self.Ki    = Ki
        self.Kd    = Kd

    def get_gains(self, error, edot):  # noqa: ARG002
        return self.Kp, self.Ki, self.Kd


YAW   = AxisParams(DT, TT, U_MIN_YAW,   U_MAX_YAW,   YAW_KP,   YAW_KI,   YAW_KD)
PITCH = AxisParams(DT, TT, U_MIN_PITCH, U_MAX_PITCH, PITCH_KP, PITCH_KI, PITCH_KD)
