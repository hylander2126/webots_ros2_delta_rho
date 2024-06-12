from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'delta_rho_sim'
path_to_share = os.path.join('share', package_name)

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join(path_to_share, 'launch'),                   glob(os.path.join('launch', '*.py'))))
data_files.append((os.path.join(path_to_share, 'worlds'),                   glob(os.path.join('worlds', '*.wbt'))))
data_files.append((os.path.join(path_to_share, 'description'),              glob(os.path.join('description', '*.*'))))
data_files.append((os.path.join(path_to_share, 'description', 'include'),   glob(os.path.join('description', 'include', '*.*'))))
data_files.append((os.path.join(path_to_share, 'protos'),                   glob(os.path.join('protos', '*.*')))) 
data_files.append((os.path.join(path_to_share, 'config'),                   glob(os.path.join('config', '*.*'))))
# data_files.append((os.path.join(path_to_share, 'resource'),                 glob(os.path.join('resource', '*.*'))))
data_files.append((path_to_share, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hylander',
    maintainer_email='stevenhyland1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = delta_rho.robot_driver:main'
        ],
    },
)
