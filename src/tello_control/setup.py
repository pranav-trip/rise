from setuptools import find_packages, setup

package_name = 'tello_control'

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
    maintainer='ptrip',
    maintainer_email='pranavtr@bu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = tello_control.test_drone:main',
            'command = tello_control.command_node:main',
            'control = tello_control.control_node:main'
        ],
    },
)
