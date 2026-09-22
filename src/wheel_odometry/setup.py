from setuptools import find_packages, setup

package_name = 'wheel_odometry'

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
    maintainer='humanoid',
    maintainer_email='humanoid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odom = wheel_odometry.odom_node:main',
            'motor_driver = wheel_odometry.motor_driver:main',
            'teleop_keyboard = wheel_odometry.teleop_keyboard:main',
            'base_controller = wheel_odometry.base_controller:main',
        ],
    },
)
