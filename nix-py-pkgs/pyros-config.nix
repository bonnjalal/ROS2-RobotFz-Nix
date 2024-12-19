{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
}:

buildPythonPackage rec {
  pname = "pyros_config";
  version = "0.2.1";
  # pyproject = true; # don't work with wheel
  format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    format = "wheel";
    # platform = "linux-x86_64"; # works with format = "wheel" only.
    sha256 = "a453b9ec058cea863b990c1eb5729e56c6e31c42112bd558267c70c57ecd1b68";
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
