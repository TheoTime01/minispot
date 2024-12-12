import os
from glob import glob
from setuptools import setup

package_name = 'quadruped_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch' ), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config' ), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'   ), glob('urdf/*')),
        (os.path.join('share', package_name, 'stl'   ), glob('stl/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        [
        'servo_controller_node = quadruped_robot.servo_controller_node:main',
        'leg_controller = quadruped_robot.leg_controller:main'
        ]
    },
)
