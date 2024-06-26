from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mystery'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
                (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michal',
    maintainer_email='37748649+MichalTesnar@users.noreply.github.com',
    description='Mystery package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'odometry_publisher = mystery.odometry_publisher:main',
                'thruster_status_publisher = mystery.thruster_status_publisher:main',
                'receiver = mystery.receiver:main',
                'inferencer = mystery.inferencer:main',
                'trainer = mystery.trainer:main',
                'evaluator = mystery.evaluator:main',
                'differentiator = mystery.differentiator:main',
        ],
},
    scripts=['scripts/uq_model.py'],
)
