from setuptools import setup

package_name = 'myrobot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teumteumee',
    maintainer_email='teumteumee@todo.todo',
    description='Child detection and guardian absence monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'child_detector   = myrobot_vision.child_detector_node:main',
            'guardian_logic   = myrobot_vision.guardian_logic_node:main',
            'tts_speaker      = myrobot_vision.tts_node:main',
            'test_publisher   = myrobot_vision.test_image_publisher:main',
            'model_mover      = myrobot_vision.model_mover_node:main',
            'child_response   = myrobot_vision.child_response_node:main',
            'image_overlay    = myrobot_vision.image_overlay_node:main',
            'patrol           = myrobot_vision.patrol_node:main',
            'auto_mapper      = myrobot_vision.auto_mapper_node:main',
        ],
    },
)
