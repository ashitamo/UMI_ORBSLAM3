from setuptools import find_packages, setup

package_name = 'umi_dataset_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab606',
    maintainer_email='lab606@example.com',
    description='Check timestamps of stereo image pairs',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collect_rgbd_bag = umi_dataset_tools.collect_rgbd_bag:main',
            'decompress_depth = umi_dataset_tools.decompress_depth:main',
            'check_stamps = umi_dataset_tools.check_stamps:main',
            'process_rgbd_bags = umi_dataset_tools.process_rgbd_bags:main',
            'visualize_gripper_rgb = umi_dataset_tools.visualize_gripper_rgb:main',
            'visualize_pointcloud_ee = umi_dataset_tools.visualize_pointcloud_ee:main',
            'visualize_orb_trajectory_speed = umi_dataset_tools.visualize_orb_trajectory_speed:main',
        ],
    },
)
