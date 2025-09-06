from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_compressed.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goldprize',
    maintainer_email='lhg9033@naver.com',
    description='Combined fire robot node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blade_node = robot.blade_node:main',
            'arm_node = robot.arm_node:main',
            'pump_node = robot.pump_node:main',
            'teleop_node = robot.teleop_node:main',
            'viewer_node = robot.viewer_node:main',
        ],
    },
)
