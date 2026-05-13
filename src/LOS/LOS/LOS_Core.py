"""
Core LOS (Line-of-Sight) path following algorithm.

Used for both horizontal (x, y) and vertical (x, z) plane guidance.
Computes the desired course angle (chi_d) that minimises cross-track error
using a fixed lookahead distance Delta [m].
"""

import numpy as np


def PathTangentAngle(x_t, y_t, x_ref, y_ref):
    """
    Calculate the path tangent angle with respect to a reference
    point (x_ref, y_ref) and target point (x_t, y_t).
    """
    pi_p = np.arctan2(y_t - y_ref, x_t - x_ref)
    return pi_p


def CrossTrackError(x, y, x_ref, y_ref, pi_p):
    """
    Calculate the cross-track error from the live AUV position (x, y) to a reference
    path defined by a reference point (x_ref, y_ref) and path tangent angle pi_p.
    """
    y_e = -(x - x_ref) * np.sin(pi_p) + (y - y_ref) * np.cos(pi_p)
    return y_e


def DesiredCourseAngle(pi_p, y_e, Delta):
    """
    Calculate the desired course angle chi_d based on the path tangent angle pi_p,
    cross-track error y_e, and fixed lookahead distance Delta.
    """
    chi_d = pi_p - np.arctan(y_e / Delta)
    return chi_d


# --------- "Step" function = one sample time ---------
def los_step(x_t, y_t, x_ref, y_ref, x, y, Delta):
    """
    One LOS update for the current vehicle position and active waypoint segment.
    Returns (chi_d [rad], pi_p [rad], y_e [m]).
    """
    pi_p  = PathTangentAngle(x_t, y_t, x_ref, y_ref)
    y_e   = CrossTrackError(x, y, x_ref, y_ref, pi_p)
    chi_d = DesiredCourseAngle(pi_p, y_e, Delta)

    return chi_d, pi_p, y_e
