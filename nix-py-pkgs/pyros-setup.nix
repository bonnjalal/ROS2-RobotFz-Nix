{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, numpy
, wheel
, pytest-runner
# tests
, pytestCheckHook
, six
, pytest
,pyros-config
}:

buildPythonPackage rec {
  pname = "pyros_setup";
  version = "0.3.0";
  pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "9c49b0daf1ed2f7e8b229a5c488369b35df51633d0f64b102d99e09323cb4bf1";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
  ];
  nativeCheckInputs = [
    pytestCheckHook
  ];
   dependencies = [
   pytest-runner
   six
   pytest
   pyros-config
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
