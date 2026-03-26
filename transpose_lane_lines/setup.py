from setuptools import find_packages, setup

package_name = 'transpose_lane_lines'

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
    maintainer='jake',
    maintainer_email='jake@todo.todo',
    description='Lane line transposition node for ROS2/Nav2 visualization',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'transpose_lines = transpose_lane_lines.transpose_lines:main',
        ],
    },
)
