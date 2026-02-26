from setuptools import setup

package_name = 'team_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team2',
    maintainer_email='brianna_ricardez@ucsb.edu',
    description='Team control nodes (teleop, mux, etc.)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = team_control.keyboard_teleop:main',
        ],
    },
)
