from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'adaptive_pivot_g2_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    test_suite='test',
    zip_safe=True,
    maintainer='Adaptive Pivot-G2 Research Team',
    maintainer_email='maintainer@example.com',
    description='Nav2 smoother comparison and execution runner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'compare_paths = adaptive_pivot_g2_benchmark.compare_paths:main',
            'batch_benchmark = adaptive_pivot_g2_benchmark.batch_benchmark:main',
            'execution_trial = adaptive_pivot_g2_benchmark.execution_trial:main',
            'execution_matrix = adaptive_pivot_g2_benchmark.execution_matrix:main',
        ],
    },
)
