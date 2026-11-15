from glob import glob
from setuptools import setup
package_name='agrisarthi_gps_nav'
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.schema.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agrisarthi Project',
    maintainer_email='maintainer@example.com',
    description='GPS waypoint navigation and acknowledged Wi-Fi UDP motor commands.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'gps_waypoint_navigator = agrisarthi_gps_nav.gps_waypoint_navigator:main',
        'mission_sampling_manager = agrisarthi_gps_nav.mission_sampling_manager:main',
        'nmea_gps_driver = agrisarthi_gps_nav.nmea_gps_driver:main',
        'heading_node = agrisarthi_gps_nav.heading_node:main',
        'simulated_esp32 = agrisarthi_gps_nav.simulated_esp32:simulated_esp32_main',
        'simulated_world = agrisarthi_gps_nav.simulated_world:simulated_world_main',
    ]},
)
