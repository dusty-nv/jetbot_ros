from setuptools import setup
from glob import glob
from itertools import chain

import os

package_name = 'jetbot_ros'

def generate_data_files():
    data_files = []
    data_dirs = ['launch', 'gazebo/worlds', 'gazebo/models']
    for path, dirs, files in chain.from_iterable(os.walk(data_dir) for data_dir in data_dirs):
        if path.startswith('gazebo/'):  # remove gazebo/ dir prefix
            install_path = path[len('gazebo/'):]
        else:
            install_path = path
        install_dir = os.path.join('share', package_name, install_path)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files
    
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dustin Franklin',
    maintainer_email='dustinf@nvidia.com',
    description='ROS nodes for JetBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
