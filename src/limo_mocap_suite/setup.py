import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'limo_mocap_suite'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib'],
    zip_safe=True,
    maintainer='Jay',
    maintainer_email='you@example.com',
    description='Hardware PhaseSpace and Controller Abstraction for LIMO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_builder = limo_mocap_suite.interactive_builder:main',
            'phasespace_bridge = limo_mocap_suite.phasespace_bridge:main',
            'pid_tracker = limo_mocap_suite.pid_tracker_node:main',
            'mpc_tracker = limo_mocap_suite.mpc_tracker_node:main',
            'lqr_tracker = limo_mocap_suite.lqr_tracker_node:main',
            'nmpc_tracker = limo_mocap_suite.nmpc_tracker_node:main',
            'live_plotter = limo_mocap_suite.live_plotter_node:main',
            'limo_test_suite = limo_mocap_suite.limo_test_suite:main',
            'hw_check = limo_mocap_suite.hw_check:main',
            'run_laps = limo_mocap_suite.run_laps:main',
            'report_lap = limo_mocap_suite.report_lap:main',
            'sim_limo = limo_mocap_suite.sim_limo:main',
            'track_publisher = limo_mocap_suite.track_publisher:main',
        ],
    },
)
