from setuptools import find_packages, setup

package_name = 'shepherding_control'

setup(
    name=package_name,
    version='0.0.0',
     packages=[
        package_name,
        f'{package_name}.my_control_library'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cinzia Tomaselli',
    maintainer_email='cinzia251996@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = shepherding_control.controller_node:main',
            'dummy_controller_node = shepherding_control.dummy_controller_node:main',
            'lama_robot_controller_node = shepherding_control.lama_robot_controller_node:main',
            'obs_robot_controller = shepherding_control.obs_robot_controller:main',
        ],
    },
)
