from setuptools import setup, find_packages
from glob import glob
from itertools import chain

import os

package_name = 'jetbot_ros'

def generate_data_files(dirs=['launch', 'gazebo/worlds', 'gazebo/models']):
    """
    Generate recursive list of data files, without listing directories in the output.
    """
    data_files = []
    for path, _, files in chain.from_iterable(os.walk(dir) for dir in dirs):
        install_dir = path[len('gazebo/'):] if path.startswith('gazebo/') else path  # remove gazebo/ prefix
        install_dir = os.path.join('share', package_name, install_dir)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files
    
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'gazebo_spawn = jetbot_ros.gazebo_spawn:main',
            'teleop_keyboard = jetbot_ros.teleop_keyboard:main',
            'data_collection = jetbot_ros.data_collection:main',
            'nav_model = jetbot_ros.nav_model:main',
        ],
    },
)
