from setuptools import setup

package_name = 'listen_talk_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/listen_talk.launch.py',
                                   'config/Hey-Willie_en_linux_v2_1_0.ppn',
                                   'config/Dog-command_en_linux_v2_1_0.rhn',
                                   'config/talk.mp3',
                                   'config/German_shepherd_barking.mp3',
                                   'config/allan.mp3',
                                   'config/easter_egg.mp3',
                                   'config/not_understanding.mp3',
                                   'config/stairs_in_front.mp3',
                                   'config/stairs_left.mp3',
                                   'config/stairs_right.mp3',
                                   'config/thank_you.mp3',
                                   'config/willie_greet.mp3']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marno Nel',
    maintainer_email='marthinusnel2023@u.northwestern.edu',
    description='Speech recognition and text to speech ROS2 package',
    license='GPvL3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listen = listen_talk_ros2.listen:main',
            'talk = listen_talk_ros2.talk:main'
        ],
    },
)
