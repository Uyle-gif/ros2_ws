from setuptools import setup
from glob import glob
import os

package_name = 'my_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # <-- dòng này copy YAML
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uylegia',
    maintainer_email='uylegia@todo.todo',
    description='Localization package (GPS + IMU + EKF + NavSat)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_imu = my_localization.fake_imu:main',
            'fake_gps = my_localization.fake_gps:main',
        ],
    },
)

