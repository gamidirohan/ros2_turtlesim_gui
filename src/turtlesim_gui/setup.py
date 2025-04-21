from setuptools import setup

package_name = 'turtlesim_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # Replace with your name
    maintainer_email='your_email@example.com',  # Replace with your email
    description='GUI for Turtlesim Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_gui = turtlesim_gui.turtlesim_gui:main',
        ],
    },
)
