from setuptools import setup

package_name = 'mrta_charge_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/charge_manager.launch.py']),
        ('share/' + package_name + '/config', ['config/stations.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Charging station reservation/queue manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'charge_manager_node = mrta_charge_manager.charge_manager_node:main',
        ],
    },
)
