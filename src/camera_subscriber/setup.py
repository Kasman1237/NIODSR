from setuptools import find_packages, setup

package_name = 'camera_subscriber'

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
    maintainer='kamil',
    maintainer_email='kamil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_subscriber.camera_node:main',
		'circle_mover = camera_subscriber.circle_mover:main',
'circle_with_point_control = camera_subscriber.circle_with_point_control:main',
'turtlebot_controller = camera_subscriber.turtlebot_controller:main',
        ],
    },
)
