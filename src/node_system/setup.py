from setuptools import find_packages, setup

package_name = 'node_system'

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
    maintainer='jangjh0201',
    maintainer_email='jangjh0201@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'VideoNode = node_system.VideoNode:main',
            'Proxy = node_system.ProxyNode:main',
            'Server = node_system.ServerNode:main',
            'camera_node = node_system.camera_node:main',
        ],
    },
)
