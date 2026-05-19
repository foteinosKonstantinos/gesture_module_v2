from setuptools import find_packages, setup

package_name = 'gesture_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harokopio University of Athens',
    maintainer_email='kfoteinos@hua.gr',
    description='Gesture recognition module. Please see the README for more details.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_classifier = gesture_recognition.classifier:main',
            'producer = gesture_recognition.producer:main'
        ]
    }
)