from setuptools import setup

package_name = 'rostak'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcos Barrera',
    maintainer_email='marcos@seafloor.com',
    description='A rostak bridge for ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rostak_bridge = rostak.rostak_bridge:main',
            'roscot_fix = rostak.roscot_fix:main',
        ],
    },
)
