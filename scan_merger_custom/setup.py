# ~/cube_ws/src/scan_merger_custom/setup.py

from setuptools import setup

package_name = 'scan_merger_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scan_merger_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Custom laser scan merger node',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'merge_scans = scan_merger_custom.merge_scans:main',
        ],
    },
)

