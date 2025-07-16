from setuptools import find_packages, setup

package_name = 'uwe_ipg_rqt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name + '/config', [
            'config/uwe_ipg_sim.perspective'
        ]),
        ('share/' + package_name + '/resource', [
            'resource/UWEIpgRobotSteeringGUI.ui'
        ]),
        
        ('share/' + package_name, ['plugin.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Jordan',
    maintainer_email='isaac_jordan@hotmail.co.uk',
    description='RQT GUIs for uwe_ipg_sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=['scripts/uwe_ipg_robot_steering_gui'],
)
