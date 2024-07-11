from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'haptic_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabriel.paffi@epfl.ch',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_command_pub = haptic_joystick.user_command_pub:main',
            'mapping = haptic_joystick.mapping_node:main',
            'user_command_pub_launch = haptic_joystick.user_command_pub_launch:main',
            'mapping_launch = haptic_joystick.mapping_node_launch:main',
            'talker = haptic_joystick.talker:main',
            'mapping_force = haptic_joystick.mapping_nodef:main',
            'mapping_same = haptic_joystick.mapping_node_same:main',


        ],
    },
)
