{ lib
, buildPythonPackage
, fetchPypi
, python312Packages
, setuptools
, numpy
}:

buildPythonPackage rec {
  pname = "cv_bridge";
  version = "1.13.0.post0";
  # pyproject = true;
  format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    format = "wheel";
    sha256 = "4e345eed822024963cab9e892f407c980e8578b0ab05f081d9ee25fb2e77996c";
  };

  propagatedBuildInputs = [ python312Packages.numpy ];
  # build-system = [
  #   setuptools
  # ];
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
