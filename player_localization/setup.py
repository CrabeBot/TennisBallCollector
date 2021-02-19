from setuptools import setup

package_name = 'player_localization'

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
    maintainer='corentin',
    maintainer_email='corentinlemoine22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p_localization = player_localization.p_localization:main',
            'p_calibration = player_localization.p_calibration:main'
        ],
    },
)
