{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
, setuptools_scm
}:

buildPythonPackage rec {
  pname = "pytest-runner";
  version = "6.0.1";
  pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "70d4739585a7008f37bf4933c013fdb327b8878a5a69fcbb3316c88882f0f49b";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
    setuptools_scm
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
