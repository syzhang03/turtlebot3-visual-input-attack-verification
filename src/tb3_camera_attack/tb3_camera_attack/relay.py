"""Message processing with an injected CvBridge, also testable without ROS."""

from copy import deepcopy

from .occlusion import black_rectangle


def process_image(message, bridge, attack_enabled=False, **rectangle):
    if not attack_enabled:
        # Preserve bytes, encoding, padding, endianness and header in Clean Mode.
        return message
    frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
    attacked = black_rectangle(frame, **rectangle)
    output = bridge.cv2_to_imgmsg(attacked, encoding='bgr8')
    output.header = deepcopy(message.header)
    return output
