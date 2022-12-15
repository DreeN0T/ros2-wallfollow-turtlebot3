from setuptools import setup

package_name = 'wallfollow'

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
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'walle = wallfollow.walle:main',
        'filter = wallfollow.filter:main',
        'controller = wallfollow.simulation_follow:main',
        'robot_controller = wallfollow.robot_follow:main',
        'better_robot_controller = wallfollow.better:main'
        ],
    },
)
