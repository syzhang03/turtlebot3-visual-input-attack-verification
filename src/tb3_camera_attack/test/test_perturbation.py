from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from tb3_camera_attack.perturbation import DEFAULTS, MODES, validate_settings, active, perturb
from tb3_camera_attack.occlusion import DEFAULT_RECT, black_rectangle
from tb3_camera_attack.relay import process_image

def cfg(**changes):
    return dict(DEFAULTS, **DEFAULT_RECT) | changes

@pytest.mark.parametrize('mode', MODES)
def test_bounds_shape_and_no_mutation(mode):
    x = np.random.default_rng(3).integers(0,256,(30,40,3),dtype=np.uint8)
    original=x.copy(); c=cfg(attack_mode=mode, noise_sigma=30., brightness_delta=-90.)
    validate_settings(c); y=perturb(x,c,np.random.default_rng(7))
    assert y.shape==x.shape and y.dtype==np.uint8
    np.testing.assert_array_equal(x,original)

def test_old_default_and_noise_reproducibility():
    x=np.full((240,320,3),100,np.uint8)
    np.testing.assert_array_equal(perturb(x,cfg(),None),black_rectangle(x))
    c=cfg(attack_mode='gaussian_noise',noise_sigma=15.)
    a=perturb(x,c,np.random.default_rng(2));b=perturb(x,c,np.random.default_rng(2))
    np.testing.assert_array_equal(a,b)
    assert not np.array_equal(a,perturb(x,c,np.random.default_rng(3)))
    np.testing.assert_array_equal(perturb(x,cfg(attack_mode='gaussian_noise'),np.random.default_rng(1)),x)

@pytest.mark.parametrize('change', [dict(noise_sigma=-1.),dict(contrast_factor=float('nan')),
    dict(attack_mode='fgsm'),dict(random_seed=-1),dict(random_seed=True),dict(period_s=0.),
    dict(on_s=3.),dict(start_s=-1.),dict(duration_s=-2.),dict(mask_x=.9),dict(shift_r=256.)])
def test_invalid_settings(change):
    with pytest.raises(ValueError):validate_settings(cfg(**change))

def test_timing_boundaries():
    c=cfg(attack_mode='intermittent_occlusion',start_s=2.,duration_s=5.)
    assert [active(c,t) for t in [1.9,2,2.99,3,4,7]]==[False,True,True,False,True,False]

def test_color_order_alpha_brightness_and_contrast():
    x=np.full((2,2,4),50,np.uint8);x[:,:,3]=123
    c=cfg(attack_mode='color_shift',shift_r=20.)
    assert list(perturb(x,c,None,'rgba8')[0,0])==[70,50,50,123]
    assert list(perturb(x,c,None,'bgra8')[0,0])==[50,50,70,123]
    assert perturb(x,cfg(attack_mode='brightness',brightness_delta=255.),None,'rgba8')[0,0,0]==255
    assert perturb(x,cfg(attack_mode='contrast',contrast_factor=0.),None,'rgba8')[0,0,0]==127
    with pytest.raises(ValueError):perturb(x[:,:,0],c,None,'mono8')

def test_header_and_preserved_encoding_bridge_contract():
    msg=SimpleNamespace(encoding='rgb8',header=SimpleNamespace(stamp=(12,34),frame_id='optical'))
    bridge=Mock();bridge.imgmsg_to_cv2.return_value=np.full((20,30,3),100,np.uint8)
    bridge.cv2_to_imgmsg.return_value=SimpleNamespace(header=None)
    original=deepcopy(msg)
    out=process_image(msg,bridge,True,settings=cfg(preserve_encoding=True))
    assert out.header==msg.header and out.header is not msg.header and msg==original
    bridge.imgmsg_to_cv2.assert_called_once_with(msg,desired_encoding='passthrough')
    assert bridge.cv2_to_imgmsg.call_args.kwargs['encoding']=='rgb8'
