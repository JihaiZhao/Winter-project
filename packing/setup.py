from setuptools import find_packages, setup

package_name = 'packing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                    'launch/realsense.launch.py',
                                    'launch/open_franka.launch.xml',
                                    'config/d435i_config.json',
                                    'config/packing.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jihai',
    maintainer_email='jihaizhao2024@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['detect_object = packing.detect_object:main',
                            # 'camera_localizer = packing.camera_localizer:main',
                            # 'pick_place = packing.pick_place:main',
                            
        ],
    },
)
