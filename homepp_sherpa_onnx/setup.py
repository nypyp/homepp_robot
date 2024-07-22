import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'homepp_sherpa_onnx'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx',
    maintainer_email='nypypbro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "speechRecognition = homepp_sherpa_onnx.speechRecognition:main",
            "keywordSpotter_from_mic = homepp_sherpa_onnx.keywordSpotterFromMic:main",
            "keywordSpotter_from_topic = homepp_sherpa_onnx.keywordSpotterFromTopic:main",
            "speechRecognition_from_topic = homepp_sherpa_onnx.speechRecognitionFromTopic:main",
        ],
    },
)
