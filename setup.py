import warnings
from setuptools import find_packages, setup

# 경고 무시
warnings.filterwarnings("ignore", category=DeprecationWarning)

package_name = 'steadylab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/ErpRead.msg']),
        ('share/' + package_name + '/msg', ['msg/ErpWrite.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bg',
    maintainer_email='okharry1@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm = steadylab.comm:main',
            'keyboard_erp = steadylab.keyboard_erp:main',
        ],
    },
)
