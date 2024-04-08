from setuptools import find_packages, setup

package_name = 'mystery'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
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
                'twist_publisher = mystery.twist_publisher:main',
                'thruster_status_publisher = mystery.thruster_status_publisher:main',
                'receive = mystery.receive:main',
                'inference = mystery.inference:main',
                'training = mystery.training:main'
        ],
},
    scripts=['scripts/model.py'],
)
