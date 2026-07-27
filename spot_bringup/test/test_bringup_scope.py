from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent
PRODUCTION_WORLDS = {
    'edgar_mine.sdf',
    'electrical_substation.sdf',
    'empty_room.sdf',
    'simple_tunnel.sdf',
}
NAVIGATION_ASSETS = (
    'launch/dlo.launch.py',
    'launch/dlo.localization.launch.py',
    'launch/dlo.mapping.launch.py',
    'launch/far_planner.launch.py',
    'launch/fast_lio_sim.launch.py',
    'launch/lio_localization.launch.py',
    'launch/local_planner.launch.py',
    'launch/planner.launch.py',
    'config/dlo_localization.rviz',
    'config/dlo_mapping.rviz',
    'config/dlo.yaml',
    'config/far_planner.yaml',
    'config/lio_localization.yaml',
    'config/localization.yaml',
    'rviz/dlo_localization.rviz',
    'rviz/dlo.rviz',
    'rviz/far_planner.rviz',
    'maps/simple_tunnel.pcd',
)


def test_sensor_adapters_are_owned_by_spot_gazebo():
    assert not (PACKAGE_ROOT / 'src/pointcloud_transform.cpp').exists()
    assert not (PACKAGE_ROOT / 'scripts/thermal_to_rgb.py').exists()
    gazebo_source = REPOSITORY_ROOT / 'spot_gazebo' / 'src'
    assert (
        gazebo_source / 'gazebo_velodyne_pointcloud_adapter.cpp'
    ).is_file()
    assert (gazebo_source / 'thermal_colormap_node.cpp').is_file()


def test_effort_smoke_test_is_owned_by_spot_gazebo_tests():
    assert not (
        PACKAGE_ROOT / 'scripts/effort_smoke_test.py'
    ).exists()
    gazebo_test = REPOSITORY_ROOT / 'spot_gazebo' / 'test'
    assert (gazebo_test / 'gazebo_effort_smoke_test.py').is_file()
    assert (gazebo_test / 'worlds/effort_smoke_test.sdf').is_file()


def test_only_supported_production_worlds_remain():
    worlds = REPOSITORY_ROOT / 'spot_gazebo' / 'worlds'
    assert {path.name for path in worlds.glob('*.sdf')} == PRODUCTION_WORLDS


def test_controller_launches_default_to_empty_room():
    launch_paths = (
        PACKAGE_ROOT / 'launch/spot.standing.launch.py',
        REPOSITORY_ROOT / 'spot_ocs2_mpc/launch/spot.mpc.launch.py',
        REPOSITORY_ROOT / 'spot_wbc/launch/spot.wbc.launch.py',
    )
    for launch_path in launch_paths:
        source = launch_path.read_text()
        assert "default_value='empty_room.sdf'" in source
        assert 'ocs2_test.sdf' not in source


def test_bridge_message_packages_have_runtime_dependencies():
    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    runtime_dependencies = {
        dependency.text for dependency in manifest.findall('exec_depend')
    }
    bridge_config = (
        PACKAGE_ROOT / 'config/spot_bridge.yaml'
    ).read_text().splitlines()
    bridge_message_packages = {
        match.group(1)
        for line in bridge_config
        if not line.lstrip().startswith('#')
        if (match := re.search(r'ros_type_name: "([^/]+)/', line))
    }
    assert bridge_message_packages <= runtime_dependencies


def test_standing_controller_has_runtime_dependency():
    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    runtime_dependencies = {
        dependency.text for dependency in manifest.findall('exec_depend')
    }
    assert 'spot_effort_controller' in runtime_dependencies


def test_navigation_assets_are_not_installed_by_simulation_bringup():
    remaining = [
        relative_path
        for relative_path in NAVIGATION_ASSETS
        if (PACKAGE_ROOT / relative_path).exists()
    ]
    assert remaining == []
