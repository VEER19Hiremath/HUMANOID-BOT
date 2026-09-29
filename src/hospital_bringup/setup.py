from setuptools import setup
import os
from glob import glob

package_name = 'hospital_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humanoid',
    maintainer_email='humanoid@todo.todo',
    description='Hospital robot bringup and Nav2 params',
    license='Apache-2.0',
)
