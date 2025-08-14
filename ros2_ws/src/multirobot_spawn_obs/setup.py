from setuptools import setup

package_name = 'multirobot_spawn_obs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_spawn_obs.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cinzia Tomaselli',
    maintainer_email='cinzia251996@gmail.com',
    description='Launch file per spawn multi robot Osoyoo + TurtleBot4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

