import importlib.util
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
LEGACY_PACKAGE_NAMES = ('champ', 'champ_base', 'champ_config', 'champ_msgs')
TEXT_SUFFIXES = {
    '.cfg', '.cpp', '.h', '.hpp', '.in', '.info', '.md', '.msg', '.py',
    '.repos', '.sdf', '.sh', '.txt', '.urdf', '.xml', '.yaml', '.yml',
}


def test_legacy_controller_packages_are_absent():
    remaining = [
        name for name in LEGACY_PACKAGE_NAMES
        if (REPOSITORY_ROOT / name).exists()
    ]
    assert remaining == []


def test_gazebo_launch_description_loads():
    launch_path = (
        REPOSITORY_ROOT / 'spot_bringup' / 'launch'
        / 'spot.gazebo.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'spot_gazebo_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.generate_launch_description() is not None


def test_repository_has_no_legacy_controller_references():
    references = []
    this_file = Path(__file__).resolve()
    for path in REPOSITORY_ROOT.rglob('*'):
        if not path.is_file() or path.resolve() == this_file:
            continue
        ignored = {'.git', 'third_party', '__pycache__'}
        if any(part in ignored for part in path.parts):
            continue
        if path.suffix not in TEXT_SUFFIXES and path.name != 'CMakeLists.txt':
            continue
        if 'champ' in path.read_text(errors='ignore').lower():
            references.append(str(path.relative_to(REPOSITORY_ROOT)))
    assert references == []
