self: super: {
  python3 = super.python3.override {
    packageOverrides = pyself: pysuper: {
      opencv4 = pysuper.opencv4.overrideAttrs (_: {
        enableGtk2 = true;
        gtk2 = pkgs.gtk2;
      });
    };
  };
}
