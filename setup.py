from setuptools import setup

package_name = 'haptic_glove_robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='artem.bisliouk@gmail.com',
    description='ROS2 package for haptic glove and robot arm simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'glove_publisher = haptic_glove_robot_arm.glove_publisher:main',
            'arm_subscriber = haptic_glove_robot_arm.arm_subscriber:main',
        ],
    },
)