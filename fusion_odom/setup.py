from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fusion_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py') if os.path.exists('launch') else []),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') if os.path.exists('config') else []),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz') if os.path.exists('rviz') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muz',
    maintainer_email='muztahid.rahman@g.bracu.ac.bd',
    description='GPS and Yaw fusion using robot_localization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher = fusion_odom.gps_publisher:main',
            'sensor_fusion_bridge = fusion_odom.sensor_fusion_bridge:main',
        ],
    },
)