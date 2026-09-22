from setuptools import setup

package_name = 'hospital_delivery'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saikirtan',
    maintainer_email='user@example.com',
    description='Hospital delivery robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'delivery_node = hospital_delivery.delivery_node:main',
            'voice_delivery_node = hospital_delivery.voice_delivery_node:main',
        ],
    },
)