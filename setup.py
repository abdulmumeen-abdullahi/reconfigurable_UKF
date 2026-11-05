import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ukf_dt_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install robot config folder and its YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install robot launch folder and its launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='microsmart',
    maintainer_email='olalekanabdulmumeen3@gmail.com',
    description='UKF-based joint state predictor for latency and noise handling',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ukf_predictor = ukf_dt_sync.ukf_predictor:main',
            'data_logger = ukf_dt_sync.data_logger:main',
            'injector = ukf_dt_sync.injector:main',
            'trajectory_executor = ukf_dt_sync.trajectory_executor:main'
        ],
    },
)
