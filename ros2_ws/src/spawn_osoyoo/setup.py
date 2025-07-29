from setuptools import setup

package_name = 'spawn_osoyoo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_robots.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TUO_NOME',
    maintainer_email='TUO_EMAIL',
    description='Descrizione del package',
    license='Licenza',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot_node = spawn_osoyoo.spawn_robot_node:main',
        ],
    },
)

