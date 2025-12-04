from setuptools import find_packages, setup

package_name = 'supervisor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'supervisor/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redknight',
    maintainer_email='redknight@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'my_first_node = supervisor.my_first_node:main',
        'my_second_node = my_second_node.my_second_node:main',
        'my_third_node = my_third_node.my_third_node:main',
        'heartbeat = supervisor.heartbeat_publisher:main',
        'supervisor = supervisor.supervisor:main',
    ],
},
)
