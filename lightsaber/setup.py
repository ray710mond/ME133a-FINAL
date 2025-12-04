from setuptools import find_packages, setup
from glob import glob

package_name = 'lightsaber'

# Create a mapping of other files to be copied in the src -> install
# build.  This is a list of tuples.  The first entry in the tuple is
# the install folder into which to place things.  The second entry is
# a list of files to place into that folder.
otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ('share/' + package_name + '/rviz', glob('rviz/*')),
    ('share/' + package_name + '/gazebo', glob('gazebo/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raymond Provost',
    maintainer_email='rprovost@caltech.edu',
    description='ME 133a Final Project',
    license='me133a',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'atlas1      = lightsaber.atlas1:main',
            'atlas2      = lightsaber.atlas2:main',
        ],
    },
)