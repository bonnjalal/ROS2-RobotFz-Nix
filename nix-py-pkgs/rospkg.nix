{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
,catkin-pkg
,pyyaml
,distro
}:

buildPythonPackage rec {
  pname = "rospkg";
  version = "1.5.1";
  pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "fce76a7477786c3732981262198ea250f097f88e0d3329c10c11069bf40ca604";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
  ];
   dependencies = [
    catkin-pkg
    pyyaml
    distro
    ];
  # dontUnpack = true;
  # installPhase = ''
  #   ${python3Packages.python} -m wheel install --no-deps --prefix=$out ${src}
  # '';

  doCheck = false;

  meta = with lib; {
    description = "ROS library to convert between ROS images and OpenCV images.";
    homepage = "https://pypi.org/project/cv-bridge/";
    license = licenses.bsd3;
  };
}
