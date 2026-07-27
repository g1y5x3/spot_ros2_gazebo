import glob
import os

from setuptools import find_packages, setup

package_name = 'spot_effort_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OCS2 Spot maintainers',
    maintainer_email='maintainers@example.com',
    description='Safe standalone effort control and Gazebo backend for Spot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'effort_backend = spot_effort_controller.backend_node:main',
            'standing_controller = spot_effort_controller.standing_node:main',
            'standing_smoke_test = '
            'spot_effort_controller.standing_smoke_test:main',
        ],
    },
)
