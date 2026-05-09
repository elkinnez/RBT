import os
from glob import glob
from setuptools import setup

package_name = 'taller_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir todos los archivos launch (Python, XML, YAML si los hubiera)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elkinnez',
    maintainer_email='elkinnez@example.com',
    description='Ejemplo de launch files con turtlesim y lanzamientos anidados',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Si en el futuro agregas nodos ejecutables, aquí los declaras
        ],
    },
)