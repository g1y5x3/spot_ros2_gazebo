import glob
import os

from setuptools import find_packages, setup

package_name = 'spot_ocs2_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/*.info') + glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OCS2 Spot maintainers',
    maintainer_email='maintainers@example.com',
    description='Spot-specific OCS2 centroidal MPC integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'observation_bridge = spot_ocs2_mpc.observation_bridge:main',
        'cmd_vel_reference = spot_ocs2_mpc.reference_node:main',
        'gait_manager = spot_ocs2_mpc.gait_node:main',
    ]},
)
