from setuptools import setup

package_name = 'waypoint_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hemanth',
    maintainer_email='hemanthvasireddy2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = waypoint_publisher.waypoint_pub:main'
            'gps_recorder = gps_recorder.gps_recorder:main'
        ],
    },
)
