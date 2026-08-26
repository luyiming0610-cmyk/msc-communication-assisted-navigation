from glob import glob

from setuptools import find_packages, setup


package_name = "epuck2_comm"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yiming Lu",
    maintainer_email="luyiming0610-cmyk@users.noreply.github.com",
    description="Lightweight neighbor-state communication library for e-puck2 robots.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_publisher = epuck2_comm.state_publisher:main",
            "state_monitor = epuck2_comm.state_monitor:main",
            "cooperative_avoider = epuck2_comm.cooperative_avoider:main",
            "network_impairment_relay = "
            "epuck2_comm.network_impairment_relay:main",
            "sequence_counter = epuck2_comm.sequence_counter:main",
            "analyze_cooperative_bag = "
            "epuck2_comm.analyze_cooperative_bag:main",
            "analyze_static_bag = epuck2_comm.analyze_static_bag:main",
            "analyze_trigger_reason = "
            "epuck2_comm.analyze_trigger_reason:main",
            "analyze_comm_performance = "
            "epuck2_comm.analyze_comm_performance:main",
        ],
    },
)
