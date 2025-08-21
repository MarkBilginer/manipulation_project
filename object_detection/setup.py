from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/object_detection.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/object_detection.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MarkBilginer',
    maintainer_email='markbilginer@gmail.com',
    description='Perception node for UR3e',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.object_detection:main',
        ],
    },
)
