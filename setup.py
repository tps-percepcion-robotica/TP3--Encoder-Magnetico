import os
from glob import glob
from setuptools import setup

package_name = 'pierna_encoder_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@ejemplo.com',
    description='TP encoder magnético AS5600 vs potenciómetro con micro-ROS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibracion_node = pierna_encoder_pkg.calibracion_node:main',
            'calibrar_potenciometro = pierna_encoder_pkg.calibrar_potenciometro:main',
            'calibracion_as5600_node = pierna_encoder_pkg.calibracion_as5600_node:main',
            'calibrar_as5600 = pierna_encoder_pkg.calibrar_as5600:main',
        ],
    },
)