from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'patrol_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabrielpb1@al.insper.edu.br',
    description='Pacote da APS2 de robótica',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = patrol_bot.patrol_node:main',
            'rotate_node = patrol_bot.rotate_node:main',
        ],
    },
)
