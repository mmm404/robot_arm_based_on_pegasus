from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pegasus_arm_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include configuration files if any
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include any URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include RViz configuration files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmms',
    maintainer_email='mmms@todo.todo',
    description='GUI commander for Pegasus Arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_gui = pegasus_arm_gui_commander.commander_gui:main',
            'pegasus_commander = pegasus_arm_commander.pegasus_commander:main',
        ],
    },
)


