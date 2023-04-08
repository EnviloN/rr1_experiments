from setuptools import setup
import os
from glob import glob

package_name = 'rr1_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='envilon',
    maintainer_email='chudyja1@fit.cvut.cz',
    description='ROS2 package for experiments performed during the development of RR1 robotic arm simulation in Unity.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "experiment_publisher_node = rr1_experiments.experiment_publisher:main",
            "experiment_subscriber_node = rr1_experiments.experiment_subscriber:main",
        ],
    },
)
