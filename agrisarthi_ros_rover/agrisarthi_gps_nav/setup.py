from setuptools import setup
package_name='agrisarthi_gps_nav'
setup(name=package_name,version='0.1.0',packages=[package_name],data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])],install_requires=['setuptools'],zip_safe=True,maintainer='Agrisarthi Project',maintainer_email='maintainer@example.com',description='GPS waypoint navigation and Wi-Fi UDP motor commands.',license='Apache-2.0',entry_points={'console_scripts':['gps_waypoint_navigator = agrisarthi_gps_nav.gps_waypoint_navigator:main']})
