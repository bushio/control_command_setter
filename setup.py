from setuptools import setup

package_name = 'control_command_setter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='satoshi',
    maintainer_email='satoshi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_setter = control_command_setter.command_setter:main',
            'controller_cmd_setter = control_command_setter.controller_cmd_setter:main',
        ],
    },
)
