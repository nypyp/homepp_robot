from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'homepp_follower'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx',
    maintainer_email='nx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yoloTracker = homepp_follower.yoloTracker:main',
            'yoloFollower = homepp_follower.yoloFollower:main'
        ],
    },
)
