from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cambiar_nombre'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Instalar carpetas
        #(os.path.join('share', package_name,'urdf'),glob('urdf/*')),
        #(os.path.join('share', package_name,'launch'),glob('launch/*.py')),
        #(os.path.join('share', package_name,'src'),glob('src/*.py')),
        #(os.path.join('share', package_name,'meshes'),glob('meshes/*')),
        #(os.path.join('share', package_name,'worlds'),glob('worlds/*')),
        #(os.path.join('share', package_name,'config'),glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nombre',
    maintainer_email='correo@gmail.com',
    description='Que hace este paquete?',
    license='Apache License 2.0',
    tests_require=['pytest'],
    
    # Instalar nodos
    #entry_points={
        #'console_scripts': [
            #'peter_controller = src.nombre_nodo:main',
        #],
    #},
)
