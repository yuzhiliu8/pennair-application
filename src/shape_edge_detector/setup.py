from setuptools import find_packages, setup
from glob import glob

package_name = 'shape_edge_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/shape_edge_detector.launch.py']),
        ('share/' + package_name + '/config', glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yliu08',
    maintainer_email='yuzhiliu8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "shape_edge_detector = shape_edge_detector.shape_edge_detector:main"
        ],
    },
)
