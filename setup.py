from setuptools import find_packages, setup

package_name = 'scan_help'

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
    maintainer='imech',
    maintainer_email='imech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "scan_node = scan_help.scan_help_now:main",
            "zero_odom = scan_help.zero_odom:main",
            "cmd_vel_relay = scan_help.cmd_vel_relay:main",
            "image_publisher = scan_help.image_publisher:main",
            "fake_camera_node = scan_help.fake_camera_node:main",
            "resize_and_scale_bbox_node = scan_help.resize_and_scale_bbox_node:main",
            "get_object_depth_node = scan_help.get_object_depth_node:main",
            "object_3d_locator = scan_help.object_3d_locator:main"
        ],
    },
)
