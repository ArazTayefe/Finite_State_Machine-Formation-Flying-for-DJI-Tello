from setuptools import setup

package_name = 'tello_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tello_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammad',
    maintainer_email='MohammadTayefeRamez@cmail.carleton.ca',
    description='The tello_driver package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_communication_node = tello_driver.tello_communication_node:main',
            'control_node = tello_driver.control_node:main',
        ],
    },
)

