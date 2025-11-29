from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Hackathon Humanoid Controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_supervisor = humanoid_control.system_supervisor:main',
            'motion_generator = humanoid_control.motion_generator:main',
            'perception_sim = humanoid_control.perception_sim:main',
            'object_spawner = humanoid_control.object_spawner:main',
        ],
    },
)
