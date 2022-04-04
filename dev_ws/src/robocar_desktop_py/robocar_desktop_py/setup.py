from setuptools import setup

package_name = 'robocar_py'

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
    maintainer='Nikita Kafanov',
    maintainer_email='nkafa2@eq.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'pca9685 = robocar_py.pca9685:main',
		'teleop_car_control_keyboard = robocar_py.teleop_car_control_keyboard:main',
		'camera_processor = robocar_py.camera_processor:main',


        ],
    },
)
