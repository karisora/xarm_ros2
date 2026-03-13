from setuptools import find_packages, setup


package_name = "autonomous_controller"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/autonomous_controller.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karisora",
    maintainer_email="karisora@example.com",
    description="ROS 2 package for executing a basic autonomous sequence from GUI start requests.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autonomous_controller_node = autonomous_controller.autonomous_controller_node:main",
        ],
    },
)
