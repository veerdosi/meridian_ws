import os
from setuptools import setup

package_name = 'meridian_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ['node_scripts/mujoco_sim_node']),
        (os.path.join('share', package_name, 'launch'), ['launch/sim_sensors.launch.py']),
        (os.path.join('share', package_name, 'scripts'), [
            'scripts/convert_urdf_to_mjcf.py',
            'scripts/augment_mjcf.py',
            'scripts/test_mjcf.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='MuJoCo simulation node for the Meridian robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
