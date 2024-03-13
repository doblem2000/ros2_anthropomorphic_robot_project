from setuptools import find_packages, setup

package_name = 'acg_resources_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','cartisian_control_msgs', 'scipy'],
    zip_safe=True,
    maintainer=['Antonio Langella', 'Michele Marsico', 'Salvatore Paolino', 'Adamo Zito'],
    maintainer_email=['a.langella31@studenti.unisa.it','m.marsico10@studenti.unisa.it','s.paolino6@studenti.unisa.it','a.zito32@studenti.unisa.it'],
    description='A robot-agnostic module for utility tools',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_analyzer = acg_resources_tools.trajectory_analyzer:main'
        ],
    },
)
