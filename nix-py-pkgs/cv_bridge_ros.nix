# nix-pkgs/cv_bridge.nix
{ lib, buildRosPackage, fetchurl, ament-cmake-gtest, ament-cmake-pytest, ament-cmake-ros,
  ament-index-python, ament-lint-auto, ament-lint-common, boost,  python-cmake-module, python3Packages, rcpputils, sensor-msgs, opencv, gtk2
}:
let
  myOpencv4 = opencv.overrideAttrs (_: {
    enableGtk2 = true;
    gtk2 = gtk2;
  });
in
buildRosPackage {
  pname = "ros-humble-cv-bridge";
  version = "3.2.1-r1";

  src = fetchurl {
    url = "https://github.com/ros2-gbp/vision_opencv-release/archive/release/humble/cv_bridge/3.2.1-1.tar.gz";
    name = "3.2.1-1.tar.gz";
    sha256 = "b4ab3dc38c288684a50c67f9fabec562bc9a3df87ca8ed2b479ebd07b28b69bb";
  };

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake-ros python-cmake-module ];
  checkInputs = [ ament-cmake-gtest ament-cmake-pytest ament-lint-auto ament-lint-common ];
  propagatedBuildInputs =  [ ament-index-python boost python3Packages.numpy rcpputils sensor-msgs myOpencv4 ];
  nativeBuildInputs = [ ament-cmake-ros python-cmake-module ];

  meta = {
    description = "This contains CvBridge, which converts between ROS2
    Image messages and OpenCV images.";
    license = with lib.licenses; [ asl20 bsdOriginal ];
  };
}
