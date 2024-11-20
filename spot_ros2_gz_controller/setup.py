from setuptools import setup

package_name = 'spot_ros2_gz_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=['spot_ros2_gz_controller'],  # Changed to match new directory name
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Spot robot controller with basic stand/sit functionality',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_controller = spot_ros2_gz_controller.spot_controller:main'
        ],
    },
)