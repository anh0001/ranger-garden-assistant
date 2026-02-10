from setuptools import setup

package_name = 'camera_lidar_fuse'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_lidar_fuse.launch.py']),
        ('share/' + package_name + '/config', [
            'config/projection_config.yaml',
            'config/camera_extrinsic_calibration.yaml',
            'config/tier4_c2_176_2880x1860.yaml',
            'config/tier4_c2_176_1440x930.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anhar Risnumawan',
    maintainer_email='anhrisn@gmail.com',
    description='Project LiDAR point clouds into camera images using saved calibration results.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projection_node = camera_lidar_fuse.camera_lidar_fuse:main',
            'projection_visualizer = camera_lidar_fuse.viz:main',
        ],
    },
)
