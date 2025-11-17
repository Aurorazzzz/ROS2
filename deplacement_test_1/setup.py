from setuptools import find_packages, setup

package_name = 'deplacement_test_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/deplacement_call.launch.py']),
        ('share/' + package_name = '/config', ['config/test_goal_publishers_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='insa',
    maintainer_email='insa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
