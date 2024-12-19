from setuptools import find_packages, setup

package_name = "robotFz_ball_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bonnjalal",
    maintainer_email="bonnjalal@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "ball_detector_node = robotFz_ball_detector.ball_detector_node:main",
            "webcam_publisher_node = robotFz_ball_detector.webcam_publisher_node:main",
        ],
    },
)
