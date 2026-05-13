# LOS_Main.py
"""
3D LOS decoupled into horizontal (x, y) and vertical plane.

The vertical channel works in a segment-relative frame where the along-track
axis always points in the direction of travel — regardless of whether the AUV
is heading north, south, east or west.  This prevents the pi_p_xz = pi (180 deg)
problem that occurred on southward segments.
"""

from dataclasses import dataclass
from .LOS_Core import los_step
import numpy as np


@dataclass
class LOSParams:
    dt: float = 0.1              # time step [s]  (kept for API compatibility)
    Delta_yaw: float = 6         # fixed lookahead distance for yaw [m]
    Delta_pitch: float = 10      # fixed lookahead distance for pitch [m]
    pitch_max_deg: float = 60.0  # hard clamp on output chi_d_xz [deg]


class LOSMain:
    """Main LOS class — usable in ROS 2 or as a standalone module."""

    def __init__(self, params: LOSParams):
        self.p = params
        self._previous_segment = None

    def _segment_key(self, x_ref, y_ref, z_ref, x_t, y_t, z_t):
        return tuple(np.round([x_ref, y_ref, z_ref, x_t, y_t, z_t], decimals=6))

    def update(self, x, y, z, x_ref, y_ref, z_ref, x_t, y_t, z_t):
        """
        One LOS update. All positions in NED [m].
        Returns dict with chi_d_xy, chi_d_xz, y_e_xy, y_e_xz,
        Delta_xy, Delta_xz, pi_p_xy, pi_p_xz.
        """
        # ----------------------------------------------------------------
        # Horizontal plane (N-E)
        # ----------------------------------------------------------------
        chi_xy, pi_p_xy, y_e_xy = los_step(
            x_t, y_t, x_ref, y_ref, x, y,
            self.p.Delta_yaw,
        )

        # ----------------------------------------------------------------
        # Vertical plane — segment-relative frame
        #
        # Project onto a 2-D frame whose x-axis points along the horizontal
        # direction of travel.  pi_p_xz stays in [-pi/2, pi/2] for any
        # compass heading, fixing the 180-deg bug on southward segments.
        #
        #   horiz_len : horizontal distance ref->target
        #   dZ        : vertical drop ref->target (NED positive = down)
        #   x_along   : AUV distance along the segment from ref
        #   z_rel     : AUV depth relative to ref
        # ----------------------------------------------------------------
        dN = x_t - x_ref
        dE = y_t - y_ref
        dZ = z_t - z_ref
        horiz_len = np.hypot(dN, dE)
        seg_len   = np.hypot(horiz_len, dZ)

        if horiz_len > 1e-6:
            ux = dN / horiz_len
            uy = dE / horiz_len
        else:
            ux, uy = 1.0, 0.0  # purely vertical segment — degenerate case

        x_along = (x - x_ref) * ux + (y - y_ref) * uy
        z_rel   = z - z_ref

        # LOS in segment-relative coordinates:
        #   target  = (seg_len, dZ)  — end of segment
        #   ref     = (0, 0)         — start of segment
        #   current = (x_along, z_rel)
        chi_xz, pi_p_xz, y_e_xz = los_step(
            seg_len, dZ, 0.0, 0.0, x_along, z_rel,
            self.p.Delta_pitch,
        )

        # Hard clamp to physical pitch limits
        pitch_max = np.deg2rad(self.p.pitch_max_deg)
        chi_xz = float(np.clip(chi_xz, -pitch_max, pitch_max))

        return {
            "chi_d_xy":  chi_xy,
            "chi_d_xz":  chi_xz,
            "y_e_xy":    y_e_xy,
            "y_e_xz":    y_e_xz,
            "Delta_xy":  self.p.Delta_yaw,
            "Delta_xz":  self.p.Delta_pitch,
            "pi_p_xy":   pi_p_xy,
            "pi_p_xz":   pi_p_xz,
        }
