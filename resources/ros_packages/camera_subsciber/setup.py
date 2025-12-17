from setuptools import setup

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nidhirajr',
    maintainer_email='nidhirajr@gmail.com',
    description='Subscriber to /camera to trigger lazy bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber_node = camera_subscriber.camera_subscriber_node:main',
        ],
    },
)
