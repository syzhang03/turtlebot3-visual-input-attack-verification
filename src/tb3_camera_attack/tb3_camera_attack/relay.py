"""Message processing with an injected CvBridge, also testable without ROS."""

from copy import deepcopy

from .occlusion import black_rectangle
from .perturbation import DEFAULTS, perturb
import numpy as np


def process_image(message, bridge, attack_enabled=False, settings=None, rng=None, **rectangle):
    if not attack_enabled or (settings is not None and settings['attack_mode'] == 'clean'):
        # Preserve bytes, encoding, padding, endianness and header in Clean Mode.
        return message
    if settings is None:
        frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        attacked = black_rectangle(frame, **rectangle)
        encoding = 'bgr8'
    else:
        encoding = message.encoding if settings['preserve_encoding'] else 'bgr8'
        frame = bridge.imgmsg_to_cv2(message, desired_encoding=(
            'passthrough' if settings['preserve_encoding'] else 'bgr8'))
        attacked = perturb(frame, settings, rng if rng is not None else np.random.default_rng(
            DEFAULTS['random_seed']), encoding)
    output = bridge.cv2_to_imgmsg(attacked, encoding=encoding)
    output.header = deepcopy(message.header)
    return output
