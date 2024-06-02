from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ora_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('scripts', package_name), glob('scripts/*.sh'))
    ],
    install_requires=['setuptools', 'odrive_can'],
    zip_safe=True,
    maintainer='Dan',
    maintainer_email='dmocnik@oakland.edu',
    description='The ORA eletrical integration package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ora_integration_test = ora_integration.integration_testing_node:main',
            'ora_integration = ora_integration.integration_node:main',
            'twist_to_wheel = ora_integration.twist_to_wheel:main',
            'wheel_to_twist = ora_integration.wheel_to_twist:main',
        ],
    },
)
