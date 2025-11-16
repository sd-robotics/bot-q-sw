import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'device_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # `rosidl_generator_py` を含める
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microphone = device_control.microphone:main',
            'speaker = device_control.speaker:main',
            'battery_monitor = device_control.battery_monitor:main',
            'keyboard_commander = device_control.keyboard_commander:main',
            'air_sensor = device_control.air_sensor:main',
            'display = device_control.display:main',
            'reaction_controller = device_control.reaction_controller:main'],
    },
)
