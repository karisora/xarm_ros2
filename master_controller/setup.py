from setuptools import find_packages, setup


package_name = "master_controller"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/master_controller.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karisora",
    maintainer_email="karisora@example.com",
    description="ROS 2 package for monitoring xArm manual mode requests.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "master_controller_node = master_controller.master_controller_node:main",
        ],
    },
)
