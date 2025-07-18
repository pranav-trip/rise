from setuptools import find_packages, setup

package_name = 'cam_sim'

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
            'sim1 = cam_sim.sim_node:main',
            'sim2 = cam_sim.top_sim:main',
            'sim3 = cam_sim.curve_sim:main',
            'sim4 = cam_sim.final_sim:main',
            'sim5 = cam_sim.turn_sim:main',
        ],
    },
)
