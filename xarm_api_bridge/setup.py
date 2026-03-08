from setuptools import find_packages, setup


package_name = "xarm_api_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/xarm_api_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karisora",
    maintainer_email="karisora@example.com",
    description="HTTP API to ROS bridge for xArm control from frontend applications.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xarm_api_bridge_server = xarm_api_bridge.server:main",
        ],
    },
)
