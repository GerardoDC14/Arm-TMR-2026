import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'dicerox_arm_urdf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (
            os.path.join('share', package_name, 'meshes', 'collision'),
            glob('meshes/collision/*'),
        ),
        (
            os.path.join('share', package_name, 'meshes', 'visual'),
            glob('meshes/visual/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerardo Escobar',
    maintainer_email='gerardodelcid16@gmail.com',
    description='Dicerox robot arm 2026',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'space_explorer_read = dicerox_arm_urdf.SpaceExplorerRead:main',
        ],
    },
)
