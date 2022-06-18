from setuptools import setup

package_name = 'robocar_desktop_py'
submodules = 'robocar_desktop_py/submodules'

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
            'camera_processor = robocar_desktop_py.camera_processor:main',
            'camera_viewer = robocar_desktop_py.camera_viewer:main',
            'heuristic_controller = robocar_desktop_py.heuristic_controller:main',
        ],
    },
)
