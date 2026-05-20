from setuptools import find_namespace_packages, setup

package_name = 'turtlebot3_Navigation_D3QN'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(include=['agent*', 'environment*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammed',
    maintainer_email='mohammed@example.com',
    description='TurtleBot3 Navigation D3QN stage-4 training and testing code',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd3qn_env = environment.d3qn_env:main',
            'd3qn_gazebo = environment.d3qn_goals:main',
            'd3qn_train = agent.d3qn_agent:main_train',
            'd3qn_test = agent.d3qn_agent:main_test',
        ],
    },
)
