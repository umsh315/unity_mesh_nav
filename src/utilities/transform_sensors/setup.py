from setuptools import setup

package_name = 'transform_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='all',
    maintainer_email='guofei@cmu.edu',
    description='Transform sensor outputs to a more "suitable" frame.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transform_everything = transform_sensors.transform_everything:main',
            'transform_hesai = transform_sensors.transform_hesai:main',
            'transform_hesai_on_pc2 = transform_sensors.transform_hesai_on_pc2:main',

            'transform_hesai_process = transform_sensors.transform_hesai_process:main',
            'sensor_fusion = transform_sensors.sensor_fusion:main',
        ],
    },
)
