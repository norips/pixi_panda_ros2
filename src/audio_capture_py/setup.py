from setuptools import find_packages, setup

package_name = 'audio_capture_py'

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
    maintainer='raphael',
    maintainer_email='alec.bossard@utoulouse.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "audio_capture = audio_capture_py.audio_capture_node:main",
            "audio_playback = audio_capture_py.audio_play_node:main",
        ],
    },
)
