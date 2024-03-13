from setuptools import find_packages, setup

package_name = 'tp_ros2'

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
    maintainer='phil',
    maintainer_email='philippe.juhel@icam.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_a_square = tp_ros2.draw_a_square:main',
            'draw_a_triangle = tp_ros2.draw_a_triangle:main'
        ],
    },
)
