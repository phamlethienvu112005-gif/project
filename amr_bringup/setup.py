from setuptools import find_packages, setup

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/real_bringup.launch.py', 'launch/slam.launch.py', 'launch/nav2.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thien Vu',
    maintainer_email='thienvu@example.com',
    description='AMR bringup package for real hardware - launch files and configurations',
    license='MIT',
)
