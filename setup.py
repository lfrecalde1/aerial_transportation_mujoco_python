from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aerial_system_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
         (os.path.join('share', package_name, 'model'), glob('model/xml/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fer',
    maintainer_email='fer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ropes_simulation = aerial_system_mujoco.ropes_simulation:main',
            'drones_simulation = aerial_system_mujoco.drones_simulation:main'
        ],
    },
)
