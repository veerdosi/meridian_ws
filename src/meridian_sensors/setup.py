import os
from setuptools import setup

package_name = 'meridian_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ['node_scripts/ft_sensor_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='F/T sensor node for the Meridian robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
