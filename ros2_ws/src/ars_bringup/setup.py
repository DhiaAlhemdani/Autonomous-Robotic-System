from setuptools import find_packages, setup

package_name = 'ars_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/heartbeat.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DhiaAlhemdani',
    maintainer_email='TODO-set-maintainer-email@example.invalid',
    description='Skeleton package: heartbeat node.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'heartbeat = ars_bringup.heartbeat:main',
        ],
    },
)
