let
  pkgs = import <nixpkgs> {};
  pkgsus = import <unstable-user> {};
in pkgs.mkShell rec {
  buildInputs = [
    pkgs.gcc
  ];
  LD_PRELOAD = "${pkgs.xorg.libX11}/lib/libX11.so ${pkgs.xorg.libX11}/lib/libX11-xcb.so ${pkgs.xorg.libXcursor}/lib/libXcursor.so ${pkgs.xorg.libXrandr}/lib/libXrandr.so ${pkgs.xorg.libXi}/lib/libXi.so ${pkgs.xorg.libxcb}/lib/libxcb.so ${pkgs.libGL}/lib/libGL.so";

}
