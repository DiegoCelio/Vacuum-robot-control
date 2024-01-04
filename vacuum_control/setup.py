from setuptools import setup

package_name = 'vacuum_control'

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
    maintainer='Diego Celio',
    maintainer_email='diego.celio@me.com',
    description='Activation of certain GPIO pin when specific keys are pressed',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_subscriber = vacuum_control.gpio_subscriber:main',
            'key_publisher = vacuum_control.key_publisher:main',
        ],
    },
)
