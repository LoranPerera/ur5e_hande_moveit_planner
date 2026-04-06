from setuptools import find_packages, setup

package_name = 'hande_bridge'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'robotiq_hande_bridge = hande_bridge.robotiq_hande_bridge:main',
                'hande_ctrl = hande_bridge.hande_ctrl:main',
                'gripper_state_splitter = hande_bridge.gripper_state_splitter:main'
        ],
    },
)
