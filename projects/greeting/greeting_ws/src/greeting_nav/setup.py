from setuptools import find_packages, setup

package_name = 'greeting_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/waypoints.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/navigate_to.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moke',
    maintainer_email='3059342114@qq.com',
    description='酒店迎宾机器人实机导航：NavigateTo 服务端，走底盘 REST :9090 固定范围短距移位',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigate_to_server = greeting_nav.navigate_to_server:main',
        ],
    },
)
