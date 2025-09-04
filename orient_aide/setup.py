from setuptools import find_packages, setup

package_name = 'orient_aide'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petalinux',
    maintainer_email='38198234+skyfty@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "musicale = orient_aide.musicale:main",
            "dac63004 = orient_aide.dac63004:main"
        ],
    },
)
