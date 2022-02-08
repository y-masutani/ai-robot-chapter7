import os
from glob import glob
from setuptools import setup


package_name = 'sample_smach'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample=sample_smach.main_sm:main',
            'sample_1=sample_smach.sample_sm:main',
            'sample_2=sample_smach.sample_sm2:main',
            'sample_3=sample_smach.sample_sm3:main',
            'manipulation=sample_smach.manipulation_node:main',
            'navigation=sample_smach.navigation_node:main',
            'vision=sample_smach.vision_node:main',
            'voice=sample_smach.voice_node:main'
        ],
    },
)
