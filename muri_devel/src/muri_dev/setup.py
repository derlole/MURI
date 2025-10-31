from setuptools import find_packages, setup

package_name = 'muri_dev'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muri_dev_team',
    maintainer_email='wirhassenapple@gmail.com',
    description='muri_development package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'muri_action_handler = muri_dev.muri_action_handler:main',
            'muri_action_server_drive = muri_dev.muri_action_server_drive:main',
            'muri_action_server_turn = muri_dev.muri_action_server_turn:main',
            'muri_action_server_init = muri_dev.muri_action_server_init:main'
        ],
    },
)
