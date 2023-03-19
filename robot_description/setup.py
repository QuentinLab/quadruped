from setuptools import setup
import os
from glob import glob


package_name = 'robot_urdf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
  	    (os.path.join('share', package_name), glob('launch/*.py')),
  	    (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('config/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quentin',
    maintainer_email='quentin.labourey@gmail.com',
    description='URDF description of the robot through XACRO file',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher=robot_urdf.joint_publisher:main'
        ],
    },
)
