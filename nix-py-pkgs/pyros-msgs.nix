{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
}:

buildPythonPackage rec {
  pname = "pyros_msgs";
  version = "0.2.0";
  pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "718484d7651ccfa93c74c030b0e104706c348e95578965ca257cedae8f2d1b50";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
  ];
   dependencies = [
    python311Packages.six
    python311Packages.pyyaml
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
