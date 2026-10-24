from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'ocean_sim'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'THIRD_PARTY.md', 'LICENSE']),
]

# Preserve nested model/material directories in the installed package.
for directory in ('launch', 'config', 'worlds', 'models'):
    files_by_parent = defaultdict(list)
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            files_by_parent[str(path.parent)].append(str(path))
    data_files.extend(
        ('share/' + package_name + '/' + parent.replace('\\', '/'), paths)
        for parent, paths in files_by_parent.items()
    )

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ocean simulation maintainers',
    maintainer_email='maintainer@example.com',
    description='Ocean waves and buoyancy for ROS 2 Humble / Gazebo Harmonic',
    license='GPL-3.0-or-later',
    entry_points={'console_scripts': [
        'vision_landing = ocean_sim.vision_landing:main',
        'landing_bootstrap = ocean_sim.landing_bootstrap:main',
    ]},
)
