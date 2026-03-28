from setuptools import setup

package_name = 'pizza_bot'

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
    maintainer='arvuser',
    maintainer_email='hardyem@umich.edu',
    description='Pizza bot controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pizza_bot_controller_node = pizza_bot.pizza_bot_controller:main'
        ],
    },
)
