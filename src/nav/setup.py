from setuptools import setup
import os
from glob import glob

package_name = 'nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zerohzzzz',
    maintainer_email='zerohzzzz0108@gmail.com',
    description='Description of your navigation package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your executable scripts here if you have any
        ],
    },
)
