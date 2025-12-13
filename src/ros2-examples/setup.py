from setuptools import setup

package_name = 'ros2_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Example Maintainer',
    maintainer_email='example@todo.todo',
    description='ROS 2 Examples for Physical AI & Humanoid Robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = publisher_subscriber.publisher:main',
            'simple_subscriber = publisher_subscriber.subscriber:main',
            'service_server = services.service_server:main',
            'service_client = services.service_client:main',
        ],
    },
)