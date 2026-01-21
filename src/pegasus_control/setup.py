from setuptools import setup
import os
from glob import glob

package_name = 'pegasus_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.gui'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Control package for Pegasus Robot Arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'headless = pegasus_control.headless_node:main',
            'gui = pegasus_control.gui.gui_node:main',
        ],
    },
)
