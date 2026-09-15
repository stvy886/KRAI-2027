from setuptools import find_packages, setup

package_name = 'auto'

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
    maintainer='stev',
    maintainer_email='stev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "modbus=auto.modbus:main",
            "nav=auto.odom:main",
            "motor=auto.motor:main",
            "deadwheel=auto.deadwheel:main"
        ],
    },
)
