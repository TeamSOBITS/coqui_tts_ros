import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'coqui_tts_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), glob('launch/tts.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tts = coqui_tts_ros.tts:main",
            "tts_action = coqui_tts_ros.tts_action:main",
        ],
    },
)
