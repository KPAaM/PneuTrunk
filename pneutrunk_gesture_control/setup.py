from setuptools import setup
from setuptools import find_packages

package_name = 'pneutrunk_gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomas',
    maintainer_email='tmerva7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pneutrunk_gesture_control = pneutrunk_gesture_control.gesture_control:main'
        ],
    },
)
