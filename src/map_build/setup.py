from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'map_build'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zerohzzzz',
    maintainer_email='zerohzzzz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_action = map_build.nav_action:main',
            'listen = map_build.listen:main',
            'demo = map_build.demo_picking:main',
            'new = map_build.dd:main',
            'follow = map_build.example_follow_path:main',
            'test = map_build.test:main'
        ],
    },
)
