import os

from generate_parameter_library_py.setup_helper import generate_parameter_module
from glob import glob
from setuptools import find_packages, setup


package_name = 'task_space_path_generator'

generate_parameter_module(
  "ouroboros_parameters", # python module name for parameter library
  "task_space_path_generator/ouroboros_parameters.yaml", # path to input yaml file
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='Antonio Langella',
    maintainer_email='a.langella31@studenti.unisa.it',
    description='A robot-agnostic module for task space path generation',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ouroboros_gui = task_space_path_generator.ouroboros_gui:main',
            'ouroboros = task_space_path_generator.ouroboros:main',
            'view_path = task_space_path_generator.view_path:main',
        ],
    },
)
