from setuptools import find_packages, setup

package_name = 'vision_proccessing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muri dev-team',
    maintainer_email='wirhassenapple@gmail.com',
    description='vision processing package for MURI robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_readout = vision_proccessing.camera_read_out:main'
        ],
    },
)
