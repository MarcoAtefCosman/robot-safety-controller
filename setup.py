from setuptools import setup
from glob import glob
import os

package_name = 'robot_safety_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), ['config/params.yaml']),
        (os.path.join('share', package_name, 'test'), glob('test/test_*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marcoatefcosmangadallah@gmail.com',
    description='Robot safety controller with state machine and behavior tree implementations',
    license='Apache License 2.0',
    tests_require=['unittest'],  
    test_suite='test',
    entry_points={
        'console_scripts': [
            'state_machine = robot_safety_controller.state_machine:main',
            'behavior_tree = robot_safety_controller.behavior_tree:main',
        ],
    },
)
