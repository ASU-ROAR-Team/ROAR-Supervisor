#src/pub_sub/setup.py
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
                ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carol',
    maintainer_email='academiccarol@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = pub_sub.publisher_member_function:main',
            'listener = pub_sub.subscriber_member_function:main',
        ],
    },
)
