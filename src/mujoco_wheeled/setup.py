from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mujoco_wheeled'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bachala Rajesh',
    maintainer_email='ravi6703@gmail.com',
    description='Standalone MuJoCo wheeled-leg robot sim node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = mujoco_wheeled.mujoco_sim_node:main',
        ],
    },
)
