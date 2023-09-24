from setuptools import find_packages, setup

package_name = 'simple_actionclient_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['simple_actionclient_py/simple_actionclient.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Intelligent Systems Lab',
    maintainer_email='isl.torvergata@gmail.com',
    description='rclpy action client, simplified.',
    license='GPL-3.0-only',
    tests_require=['pytest']
)
