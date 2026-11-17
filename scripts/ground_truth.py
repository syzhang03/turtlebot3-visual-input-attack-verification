"""Evaluation-only conservative geometry; never used by the controller."""
import math
import xml.etree.ElementTree as ET


def collision_radius(model_file):
    """Enclose every collision primitive in a model-origin 3D sphere.

    Triangle inequality makes this bound independent of local rotations.
    Unknown geometry/relative frames must not silently produce a smaller bound.
    """
    model = ET.parse(model_file).getroot().find('model')
    radii = []
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            offsets = []
            for element in (link, collision):
                pose = element.find('pose')
                if pose is not None and pose.get('relative_to'):
                    raise ValueError('Relative pose frames require explicit resolution')
                offsets.append(math.dist([0, 0, 0], [float(v) for v in pose.text.split()[:3]])
                               if pose is not None else 0.0)
            geometry = collision.find('geometry')
            if geometry.find('box') is not None:
                radius = math.sqrt(sum(float(v)**2 for v in geometry.findtext('box/size').split()))/2
            elif geometry.find('cylinder') is not None:
                radius = math.hypot(float(geometry.findtext('cylinder/radius')),
                                    float(geometry.findtext('cylinder/length'))/2)
            elif geometry.find('sphere') is not None:
                radius = float(geometry.findtext('sphere/radius'))
            else:
                raise ValueError('Unsupported collision geometry')
            radii.append(sum(offsets)+radius)
    if not radii:
        raise ValueError('No collision primitives found')
    return max(radii)


def box_clearance_lower_bound(x, y, radius, box_x=4.0, box_y=0.0, half_x=0.15, half_y=0.25):
    dx = max(abs(x-box_x)-half_x, 0.0)
    dy = max(abs(y-box_y)-half_y, 0.0)
    return math.hypot(dx, dy)-radius
