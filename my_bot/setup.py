from setuptools import setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('lib/' + package_name, glob('scripts/*')),  # Ensure scripts are included
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kunal',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ignore_lidar_values = my_bot.ignore_lidar_values:main',
        ],
    },
)

