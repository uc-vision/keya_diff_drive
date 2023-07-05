from setuptools import setup

package_name = 'keya_diff_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mma484',
    maintainer_email='matthew.mattar@canterbury.ac.nz',
    description='Twist driver used to communicate with the motors',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_motor_node = keya_diff_drive.twist_to_motor_node:main'
        ],
    },
)
