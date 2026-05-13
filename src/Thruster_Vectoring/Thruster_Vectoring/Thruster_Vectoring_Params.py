import math

# Topics
YAW_PITCH_TOPIC   = "/pid/cmd_yaw_pitch"
RC_OVERRIDE_TOPIC = "/mavros/rc/override"

# RC override channels 
THRUSTER_RC_CH = 1
PITCH_RC_CH    = 9
YAW_RC_CH      = 11

# Thruster control
CONTROL_THRUSTER = True
THRUSTER_PWM     = 1750
THRUSTER_NEUTRAL = 1500

# Timing and safety
UPDATE_HZ        = 50.0
CMD_TIMEOUT_S    = 10.0
SLEW_PWM_PER_SEC = 800.0

# PWM limits
PWM_MIN = 800
PWM_MAX = 2200

# Axis signs 
PITCH_SIGN = 1.0
YAW_SIGN   = -1.0

# Max physical angles
PITCH_MAX_RAD = math.radians(16.0)
YAW_MAX_RAD   = math.radians(16.0)


# Lookup tables: angle [deg] -> PWM
PITCH_LUT_DEG = [-16.0, -12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0, 16.0]
PITCH_LUT_PWM = [800, 975, 1150, 1325, 1500, 1675, 1850, 2025, 2200]

YAW_LUT_DEG = [-16.0, -12.0, -8.5, -4.0, 0.0, 4.0, 8.5, 12.0, 16.0]
YAW_LUT_PWM = [800, 975, 1150, 1325, 1500, 1675, 1850, 2025, 2200]
