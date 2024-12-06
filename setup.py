from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'cafe_root'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob('launch_file/bulter.launch.py')),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vignesh',
    maintainer_email='vignesh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = cafe_root.cafe_publisher:main',
            'test_sub = cafe_root.cafe_subscrier:main',
            'test_chef = cafe_root.kitchen_node:main',
            
        ],
    },
)
