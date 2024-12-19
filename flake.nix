{
  description = "A very basic flake";


  inputs = {
    # nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-24.11";
    nixpkgs-python.url = "github:cachix/nixpkgs-python";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };

  outputs = { self, nixpkgs, nixpkgs-python, nix-ros-overlay }:
  nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
    let
      # pkgs = nixpkgs.legacyPackages.x86_64-linux;
      shell_name = "Nix-RoboFz";
      system = "x86_64-linux";
      pkgs = import nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [ nix-ros-overlay.overlays.default ];
            # config.cudaSupport = true;
            # config.cudaVersion = "12";
          };
      # pkg-py = nixpkgs-python.packages.x86_64-linux;
      python = let
        packageOverrides = self: super: {
        # packageOverrides = pyfinal: pyprev: {
          opencv4 = super.opencv4.override {
            enableGtk2 = true;
            gtk2 = pkgs.gtk2;
            # enableGtk3   = true; 
            # enableFfmpeg = true; 
            # ffmpeg = pkgs.ffmpeg-full;
            # enableCuda   = true;
            # enableUnfree = true;
            #ffmpeg_3 = pkgs.ffmpeg-full;
            };

          # cv_bridge = self.callPackage ./nix-py-pkgs/cv_bridge.nix { };
          # rospy2 = self.callPackage ./rospy2.nix { };
          # pyros-msgs = pyfinal.callPackage ./pyros-msgs.nix { };
          # pytest-runner = pyfinal.callPackage ./pytest-runner.nix { };
          # pyros-setup = pyfinal.callPackage ./pyros-setup.nix { };
          # pyrosenv = pyfinal.callPackage ./pyrosenv.nix {gnupg = pkgs.gnupg; };
          # pyros-config = pyfinal.callPackage ./pyros-config.nix { };
          # rospkg = pyfinal.callPackage ./rospkg.nix { };

        };
        in
          pkgs.python312.override {
            inherit packageOverrides;
            self = python;
          };

    in
    {

      # packages.x86_64-linux.hello = pkgs.hello;

        # hardware.nvidia.package = pkgs.config.boot.kernelPackages.nvidiaPackages.legacy_470;

      # devShells.x86_64-linux.default = pkgs.mkShell {
      devShells.default = pkgs.mkShell {

        name = shell_name;
        shellName = shell_name;
        # hardware.nvidia.package = config.boot.kernelPackages.nvidiaPackages.legacy_470;
        NIX_LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
          pkgs.stdenv.cc.cc
          pkgs.zlib
          pkgs.libGL
          pkgs.glib
          pkgs.openssl_3_2
          pkgs.rosPackages.humble.rmw-fastrtps-cpp
          pkgs.rosPackages.humble.ament-cmake-core

          # pkgs.kdePackages.qtwayland

        ];
        NIX_LD = pkgs.lib.fileContents "${pkgs.stdenv.cc}/nix-support/dynamic-linker";

        # nativeBuildInputs = with pkgs; [
        #     libsForQt5.qt5.qmake
        #
        #     # For setting Qt environment variables.
        #     qt5.wrapQtAppsHook
        #     makeWrapper
        # ];
        # dontWrapQtApps = true;

        # cv_bridge = pkgs.rosPackages.humble.cv-bridge.overrideAttrs (oldAttrs: rec {
        #   propagatedBuildInputs =
        #     pkgs.lib.filter (p: p != pkgs.opencv && p !=  pkgs.python3Packages.opencv4) oldAttrs.propagatedBuildInputs
        #       ++ [  python.opencv4  pkgs.python3Packages.numpy  pkgs.rcpputils pkgs.sensor-msgs ];
        #
        # });
        packages = [
              pkgs.colcon

              # ... other non-ROS packages
          (with pkgs.rosPackages.humble; buildEnv {
                paths = [
                  ros-base
                  rqt-graph
                  # cv-bridge
                  rmw
                  rmw-fastrtps-cpp
                  ament-cmake-core
                  # ... other ROS packages
                  # (callPackage ./nix-py-pkgs/cv_bridge_ros.nix {opencv4 = pkgs.python312Packages.opencv4; })
                  # myCvBridge
                  (callPackage ./nix-py-pkgs/cv_bridge_ros.nix { })

                ];
              })
        ];

        buildInputs = with pkgs; [


          # pkgs.colcon

          # ... other non-ROS packages
          # glibcLocales
          # (with pkgs.rosPackages.noetic; buildEnv {
          #   paths = [
          #     ros-base
          #     # desktop-full
          #     # ... other ROS packages
          #   ];
          # })


          # pkg-py."3.11"
          # python312
          # pkgs.libsForQt5.qt5.qtserialport
          # pkgs.gtk3
          # pkgs.kdePackages.qtwayland
          # python312Packages.numpy
          # pkgs.python312Packages.opencv4

          # python.opencv4
          (python.withPackages(ps: with ps; [
              opencv4
              # cv_bridge
              # rospy2
              # pyrosenv
              # pytest-runner
              # pyros-setup
              # pyros-config
              # pyros-msgs
              # rospkg

              numpy

              rosdep
          ]))
          # callPackage ./cv_bridge.nix { }
        # (pkgs.callPackage ./opencv.nix { })

        ];

        # this is for ros
        ROS_HOSTNAME = "localhost";
        ROS_MASTER_URI = "http://localhost:11311";
        TURTLEBOT3_MODEL = "burger";

        shellHook = ''
          export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH

          # current_path="$PWD"
          # folder_path="$current_path/$shellName"
          #
          # if [ ! -d "$folder_path" ]; then
          #   echo "Creating vertual env: $folder_path"
          #   python -m venv $shellName
          # else
          #   echo "vertual env '$folder_path' already exists."
          # fi

          # source $shellName/bin/activate

          ## This is if we are not using python env
          VIRTUAL_ENV="$PWD/$shellName"
          export VIRTUAL_ENV

          export QT_QPA_PLATFORM=xcb

          source src/setup.bash

          # source shell.sh $shellName
          # export PS1


          '';
      };

  });

  nixConfig = {
    #   substituters = [
    #   "https://mirrors.ustc.edu.cn/nix-channels/store"
    #   # "https://cache.nixos.org"
    #   # "https://hydra.nixos.org"
    # ];

    extra-substituters = [ 
      "https://ros.cachix.org"
      "https://nix-community.cachix.org" 
    ];
    extra-trusted-public-keys = [
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY=" 
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

}
