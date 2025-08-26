from setuptools import setup

package_name = 'mrta_executor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/executor_tb3_0.launch.py',
            'launch/executor_tb3_1.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot executor (Nav2 NavigateToPose + charge reserve/release)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'executor_node = mrta_executor.executor_node:main',
        ],
    },
)
