from setuptools import find_packages, setup

package_name = 'koch_wrapper'

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
    maintainer='pomelo925',
    maintainer_email='yoseph.huang@gmail.com',
    description='ROS2 wrapper for Koch robot arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ctrl_leader_follower = koch_wrapper.ctrl_leader_follower:main',
            'record = koch_wrapper.record:main',
        ],
    },
)
