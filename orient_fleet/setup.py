from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'orient_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='petalinux',
    maintainer_email='38198234+skyfty@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=orient_fleet.fleet_adapter:main',
            'fleet_manager=orient_fleet.fleet_manager:main',
            'fleet_mqtt_bridge=orient_fleet.fleet_mqtt_bridge:main'
        ],
    },
)
