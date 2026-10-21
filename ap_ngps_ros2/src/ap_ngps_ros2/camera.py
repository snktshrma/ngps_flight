"""The one place the flight camera's intrinsics live (SIYI ZR10 as mounted).

Both live nodes take their defaults from here and the per-map configs no longer
repeat them, so a map config cannot quietly carry a stale value again: flight 2's
Maxar config still said HFOV 71.8 deg after the 49.2 measurement, which oversized
every map crop 1.6x. config_check rejects any config that overrides these with a
different value; change them here, and only after measuring.
"""

# Horizontal field of view at the 640 px working width, measured on flight 1.
HFOV_DEG = 49.2
# Image "up" relative to the vehicle's heading.
YAW_OFFSET_DEG = 1.3
# The camera points ~3 deg off nadir: the pixel below the aircraft, as a fraction of
# the image size from the centre (+x right, +y down). Fitted on flight 1's climb and
# checked on its descent.
NADIR_OFFSET_X_FRAC = 0.05757
NADIR_OFFSET_Y_FRAC = -0.00184

# Parameter name -> canonical value, per node.
PARAMS = {
    'vo_ngps_fusion_node': {
        'hfov_deg': HFOV_DEG,
        'camera_yaw_offset_deg': YAW_OFFSET_DEG,
        'camera_nadir_offset_x_frac': NADIR_OFFSET_X_FRAC,
        'camera_nadir_offset_y_frac': NADIR_OFFSET_Y_FRAC,
    },
    'ngps_localization_node': {
        'kernel_hfov_deg': HFOV_DEG,
        'camera_yaw_offset_deg': YAW_OFFSET_DEG,
        'camera_nadir_offset_x_frac': NADIR_OFFSET_X_FRAC,
        'camera_nadir_offset_y_frac': NADIR_OFFSET_Y_FRAC,
    },
}
