"""Shared parameter validation for the four experiment conditions."""
import math


def condition(mode, camera_delay_s=1.0, lidar_bias_m=1.0, sector_half_width_deg=30.0,
              duration_s=600.0):
    if mode not in ('clean', 'camera', 'lidar', 'joint'):
        raise ValueError('mode must be clean, camera, lidar or joint')
    names = ('camera_delay_s', 'lidar_bias_m', 'sector_half_width_deg', 'duration_s')
    values = dict(zip(names, map(float, (camera_delay_s, lidar_bias_m, sector_half_width_deg, duration_s))))
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError('Parameters must be finite numbers')
    if values['camera_delay_s'] < 0 or values['lidar_bias_m'] < 0:
        raise ValueError('Delay and bias must be nonnegative')
    if not 0 < values['sector_half_width_deg'] <= 180 or values['duration_s'] <= 0:
        raise ValueError('Require 0 < sector half width <= 180 and duration > 0')
    if mode in ('camera', 'joint') and values['camera_delay_s'] == 0:
        raise ValueError('Camera attack condition requires a positive delay; use clean/lidar otherwise')
    if mode in ('lidar', 'joint') and values['lidar_bias_m'] == 0:
        raise ValueError('LiDAR attack condition requires a positive bias; use clean/camera otherwise')
    return dict(mode=mode, requested=values, effective=dict(
        camera_delay_s=values['camera_delay_s'] if mode in ('camera', 'joint') else 0.0,
        lidar_bias_m=values['lidar_bias_m'] if mode in ('lidar', 'joint') else 0.0,
        sector_center_deg=0.0, sector_half_width_deg=values['sector_half_width_deg'],
        start_s=0.0, duration_s=values['duration_s'],
    ))
