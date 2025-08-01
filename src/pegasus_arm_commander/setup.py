from setuptools import setup

package_name = 'pegasus_arm_commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam',
    maintainer_email='sam@example.com',
    description='Command interface for Pegasus robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pegasus_commander = pegasus_arm_commander.pegasus_commander:main',
        ],
    },
)

