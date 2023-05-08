import setuptools
from pathlib import Path

setuptools.setup(
    name='rover_arm',
    version='1.1.6',
    description="A OpenAI Gym Env for Rover with Arm",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    author="Sai Phani",
    packages = setuptools.find_packages(include="rover_arm*"),
    package_data={'data' :['rover_arm/data/*']},
    include_package_data=True,
    install_requires=['gymnasium', 'pybullet', 'gym'],  # And any other dependencies foo needs
    extras_require = {
        'dev':  ['pynput']
    }
)
