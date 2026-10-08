from setuptools import find_packages, setup

package_name = 'greeting_body'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/joint_map.yaml',
            'config/motions.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/motion.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moke',
    maintainer_email='3059342114@qq.com',
    description='酒店迎宾机器人实机礼仪动作库：PlayMotion 服务端，流式下发头/臂/腰关节帧',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_server = greeting_body.motion_server:main',
        ],
    },
)