from setuptools import find_packages, setup

package_name = 'inverted_pendulum_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/inverted_pendulum_control']),
        ('share/inverted_pendulum_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='kaan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_node = inverted_pendulum_control.control_node:main',
            'disturbance_test = inverted_pendulum_control.disturbance_test:main',
        ],
    },
)
