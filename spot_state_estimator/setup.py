import glob
import os

from setuptools import find_packages, setup

package_name = 'spot_state_estimator'
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
    description='Replaceable simulation state adapter for Spot OCS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'state_adapter = spot_state_estimator.state_adapter_node:main',
    ]},
)
