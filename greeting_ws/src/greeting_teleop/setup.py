from setuptools import find_packages, setup

package_name = 'greeting_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/teleop_map.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/teleop.launch.py',
            'launch/greeting_bringup.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moke',
    maintainer_email='3059342114@qq.com',
    description='遥控器(SBUS)按键网关：把 /sbus_data 的按键/拨杆映射为迎宾命令，发布到 /greeting/panel_command',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_mapper = greeting_teleop.joy_mapper:main',
            'panel_command_bridge = greeting_teleop.panel_command_bridge:main',
            'flow_command_bridge = greeting_teleop.flow_command_bridge:main',
        ],
    },
)