from setuptools import setup, find_packages

package_name = 'sensor_fault_injection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/all_faults.launch.py',
            'launch/gps_fault.launch.py',
            'launch/imu_fault.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/fault_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Sensor Fault Injection Package for VRX USV Simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_fault_injection_node = sensor_fault_injection.gps_fault_injection_node:main',
            'imu_fault_injection_node = sensor_fault_injection.imu_fault_injection_node:main',
        ],
    },
)
