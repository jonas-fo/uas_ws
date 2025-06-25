from setuptools import find_packages, setup

package_name = 'thrust_package'

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
    maintainer='uaskash',
    maintainer_email='uaskash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_command = thrust_package.arm_command:main',
            'thrust = thrust_package.thrust_command:main',
            'position_converter = thrust_package.position_converter:main'
        ],
    },
)
