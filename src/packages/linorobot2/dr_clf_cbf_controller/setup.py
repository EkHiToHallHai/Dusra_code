from setuptools import setup
import os
from glob import glob

package_name = 'dr_clf_cbf_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        
        ('share/' + package_name + '/config', 
            ['config/controller_config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='CLF-CBF controller',
    license='BSD',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dr_cbf_controller_node = dr_clf_cbf_controller.dr_cbf_controller:main',
            'waypoint_navigator      = dr_clf_cbf_controller.waypoint_navigator:main',
        ],
    },
)