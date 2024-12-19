{ lib
, buildPythonPackage
, fetchPypi
, python311Packages
, setuptools
, pyros-setup
, rospkg
,defusedxml
,pycrypto
,gnupg
# ,python-gnupg
# , numpy
}:

buildPythonPackage rec {
  pname = "pyrosenv";
  version = "0.0.4";
  # pyproject = true;
  # format = "wheel";
  
  src = fetchPypi {
    inherit pname version;
    # format = "wheel";
    sha256 = "90e0d90becd328d99aa83667212b352263ef398133792497247fad1780b6f7f0";
  };

  # propagatedBuildInputs = [ python311Packages.numpy ];
  build-system = [
    setuptools
  ];
  dependencies = [
    pyros-setup
    rospkg
    defusedxml
    pycrypto
    gnupg
    # python-gnupg
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
