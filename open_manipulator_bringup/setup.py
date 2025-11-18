from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_bringup'

setup(
    name=package_name,
    version='4.1.0',
    packages=find_packages(exclude=['test']), # open_manipulator_bringup/ 폴더 안의 Python 모듈들을 자동으로 찾아서 패키지로 등록
    data_files=[
        # ROS2는 패키지별로 필요한 파일을 share/<패키지 이름>/ 아래에 모아두는 규칙이 있음
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 폴더에 있는 모든 py 파일 가져와서 share/open.../launch 폴더에 다운
        # 아래도 동일함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config/open_manipulator_x'),
            glob('config/open_manipulator_x/*')),
        (os.path.join('share', package_name, 'config/omy_3m'), glob('config/omy_3m/*')),
        (os.path.join('share', package_name, 'config/omy_f3m'), glob('config/omy_f3m/*')),
        (os.path.join('share', package_name, 'config/omy_f3m_follower_ai'),
            glob('config/omy_f3m_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omy_f3m_leader_ai'),
            glob('config/omy_f3m_leader_ai/*')),
        (os.path.join('share', package_name, 'config/omy_l100_follower_ai'),
            glob('config/omy_l100_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omy_l100_leader_ai'),
            glob('config/omy_l100_leader_ai/*')),
        (os.path.join('share', package_name, 'config/open_manipulator_x'),
            glob('config/open_manipulator_x/*')),
        (os.path.join('share', package_name, 'config/omx_f'),
            glob('config/omx_f/*')),
        (os.path.join('share', package_name, 'config/omx_f_follower_ai'),
            glob('config/omx_f_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omx_l_leader_ai'),
            glob('config/omx_l_leader_ai/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name), ['open-manipulator-cdc.rules']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='OpenMANIPULATOR bringup ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],

    # ros2 명령어 간소화
    # joint_trajectory_executor -> open_manipulator_bringup/joint_trajectory_executor.py/main 함수 호출하겠다
    entry_points={
        'console_scripts': [
            'joint_trajectory_executor = open_manipulator_bringup.joint_trajectory_executor:main',
            'om_create_udev_rules = open_manipulator_bringup.om_create_udev_rules:main',
        ],
    },
)
