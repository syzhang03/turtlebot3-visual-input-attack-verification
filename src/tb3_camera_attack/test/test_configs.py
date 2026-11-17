from pathlib import Path
import ast
import yaml
from tb3_camera_attack.perturbation import DEFAULTS, validate_settings
from tb3_camera_attack.occlusion import DEFAULT_RECT

def test_all_presets_valid_and_integration_output():
    package=Path(__file__).resolve().parents[1]
    for path in (package/'config').glob('*.yaml'):
        parameters=yaml.safe_load(path.read_text())['camera_occlusion']['ros__parameters']
        settings=dict(DEFAULTS,**DEFAULT_RECT)
        settings.update({k:v for k,v in parameters.items() if k in settings})
        validate_settings(settings)
        if path.stem!='camera_occlusion':assert parameters['output_topic']=='/attack/image_raw'

def test_attack_only_publishes_image_type():
    package=Path(__file__).resolve().parents[1]
    tree=ast.parse((package/'tb3_camera_attack'/'node.py').read_text(encoding='utf8'))
    publishers=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
                and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
    assert len(publishers)==1 and isinstance(publishers[0].args[0],ast.Name)
    assert publishers[0].args[0].id=='Image'
