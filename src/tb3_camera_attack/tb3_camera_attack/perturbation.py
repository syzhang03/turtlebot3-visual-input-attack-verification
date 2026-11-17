"""Digital pixel perturbations, not physical attacks or gradient attacks."""
import math
import numpy as np
from .occlusion import DEFAULT_RECT, validate_rectangle

DEFAULTS = dict(attack_mode='occlusion', start_s=0.0, duration_s=-1.0,
                period_s=2.0, on_s=1.0, noise_sigma=0.0, random_seed=0,
                brightness_delta=0.0, contrast_factor=1.0,
                shift_b=0.0, shift_g=0.0, shift_r=0.0, preserve_encoding=False)
MODES = ('clean', 'occlusion', 'intermittent_occlusion', 'gaussian_noise',
         'brightness', 'contrast', 'color_shift')


def validate_settings(cfg):
    validate_rectangle(**{k: cfg[k] for k in DEFAULT_RECT})
    if cfg['attack_mode'] not in MODES:
        raise ValueError('Unknown attack_mode')
    if type(cfg['preserve_encoding']) is not bool:
        raise ValueError('preserve_encoding requires bool')
    if type(cfg['random_seed']) is not int or not 0 <= cfg['random_seed'] < 2**32:
        raise ValueError('random_seed must be an integer in [0, 2**32)')
    for k in ('start_s', 'duration_s', 'period_s', 'on_s', 'noise_sigma',
              'brightness_delta', 'contrast_factor', 'shift_b', 'shift_g', 'shift_r'):
        if type(cfg[k]) not in (float, int) or not math.isfinite(cfg[k]):
            raise ValueError(f'{k} must be finite')
    if cfg['start_s'] < 0 or (cfg['duration_s'] < 0 and cfg['duration_s'] != -1):
        raise ValueError('start_s >= 0; duration_s >= 0 or -1 for unlimited')
    if not 0 < cfg['on_s'] <= cfg['period_s']:
        raise ValueError('0 < on_s <= period_s required')
    if not 0 <= cfg['noise_sigma'] <= 255 or not 0 <= cfg['contrast_factor'] <= 10:
        raise ValueError('sigma in [0,255], contrast in [0,10]')
    if any(abs(cfg[k]) > 255 for k in ('brightness_delta', 'shift_b', 'shift_g', 'shift_r')):
        raise ValueError('Brightness/channel shifts must be within [-255,255]')


def active(cfg, elapsed):
    t = elapsed - cfg['start_s']
    return (cfg['attack_mode'] != 'clean' and t >= 0
            and (cfg['duration_s'] == -1 or t < cfg['duration_s'])
            and (cfg['attack_mode'] != 'intermittent_occlusion'
                 or t % cfg['period_s'] < cfg['on_s']))


def perturb(frame, cfg, rng, encoding='bgr8'):
    if frame.dtype != np.uint8 or frame.size == 0:
        raise ValueError('Only nonempty 8-bit images are supported')
    expected = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
    if encoding not in expected or (frame.ndim != 2 if encoding == 'mono8'
                                    else frame.ndim != 3 or frame.shape[2] != expected[encoding]):
        raise ValueError('Unsupported encoding or image shape')
    result = frame.copy()
    pixels = result if frame.ndim == 2 else result[:, :, :3]  # Preserve alpha.
    mode = cfg['attack_mode']
    if mode in ('occlusion', 'intermittent_occlusion'):
        h, w = frame.shape[:2]
        x, y, rw, rh = (cfg[k] for k in DEFAULT_RECT)
        pixels[int(h*y):min(h, int(h*(y+rh))), int(w*x):min(w, int(w*(x+rw)))] = 0
    elif mode != 'clean':
        values = pixels.astype(np.float64)
        if mode == 'gaussian_noise':
            values += rng.normal(0, cfg['noise_sigma'], pixels.shape)
        elif mode == 'brightness':
            values += cfg['brightness_delta']
        elif mode == 'contrast':
            values = (values - 127.5) * cfg['contrast_factor'] + 127.5
        elif mode == 'color_shift':
            if encoding == 'mono8':
                raise ValueError('color_shift requires a color encoding')
            shift = [cfg['shift_b'], cfg['shift_g'], cfg['shift_r']]
            values += shift[::-1] if encoding.startswith('rgb') else shift
        pixels[...] = np.clip(values, 0, 255).astype(np.uint8)
    return result
