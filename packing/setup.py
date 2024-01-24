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
                                    'launch/realsense.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jihai',
    maintainer_email='jihaizhao2024@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['cup_detection = packing.test:cup_detection_entry',
                            'detect_object = packing.detect_object:main'
        ],
    },
)
