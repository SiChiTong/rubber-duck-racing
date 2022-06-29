from setuptools import setup

package_name = 'rdrpy'
submodules = 'rdrpy/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikita',
    maintainer_email='nikita@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = rdrpy.motor_driver:main',
            'hsv_cam = rdrpy.hsv_cam:main',
            'camera_processor = rdrpy.camera_processor:main',
            'camera_viewer = rdrpy.camera_viewer:main',
            'heuristic_controller = rdrpy.heuristic_controller:main',
        ],
    },
)
