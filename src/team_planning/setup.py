from setuptools import setup

package_name = 'team_planning'

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
    description='Team planning stack (FTG / Pure Pursuit / SLAM later)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ftg = team_planning.ftg_node:main',
        ],
    },
)
