from setuptools import setup
import os
from glob import glob

package_name = 'cps_loki_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), #[1:]
        (os.path.join('share', package_name, 'maps'), glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bjorn',
    maintainer_email='bjoern.ellensohn@gmail.com',
    description='Launch Files for CPS Robot Mini',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
