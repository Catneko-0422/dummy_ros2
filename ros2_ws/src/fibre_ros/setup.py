from setuptools import setup, find_packages
from glob import glob

package_name = 'fibre_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['fibre_ros', 'fibre', 'fibre.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyusb'],  # fibre 會用到 usb，系統層也要 libusb
    zip_safe=True,
    maintainer='nekocat',
    maintainer_email='nekocat@example.com',
    description='ROS2 wrapper for fibre device discovery',
    license='MIT',
    entry_points={
        'console_scripts': [
            'discover = fibre_ros.discover_node:main',
        ],
    },
)

