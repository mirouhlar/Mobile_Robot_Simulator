from setuptools import setup

package_name = 'sim_pkg'

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
   maintainer='MiroslavUhlar',
    maintainer_email='miroslav.uhlar@student.tuke.sk',
    description='Task1 - Navigation methods in robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
   entry_points={
        'console_scripts': [
                'simulator = sim_pkg.simulator:main',
        	'sender = sim_pkg.map_publish:main',
        	'subscriber = sim_pkg.map_subscribe:main',

        ],
    },
)
