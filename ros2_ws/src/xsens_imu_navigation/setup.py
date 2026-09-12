from glob import glob
from setuptools import find_packages, setup

package_name = 'xsens_imu_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='lab606',
    maintainer_email='lab606@example.com',
    description='Short-horizon Xsens free-acceleration dead reckoning with ZUPT.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_dead_reckoning = xsens_imu_navigation.imu_dead_reckoning_node:main',
        ],
    },
)
