from setuptools import find_packages, setup

package_name = 'transitions_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['transitions_ros/state.py', 'transitions_ros/machine.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intelligent Systems Lab',
    maintainer_email='isl.torvergata@gmail.com',
    description='Finite-state machine with ROS 2 capabilities.',
    license='GPL-3.0-only',
    tests_require=['pytest']
)
