{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
}:

buildPythonPackage rec {
  pname = "rospy2";
  version = "1.0.3";
  pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "2c128e94a5e2ace1ec4b7aafd15c540dbd33276f11677697bdf83409ad18452c";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
  ];
   # dependencies = [
    # numpy
    # python311Packages.numpy
  # ];
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
