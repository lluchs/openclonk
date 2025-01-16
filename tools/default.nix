{ pkgs ? import <nixpkgs> {} , withEditor ? false , withMape ? false }:

pkgs.stdenv.mkDerivation {
  name = "openclonk";

  gitRef = pkgs.lib.commitIdFromGitRepo ../.git;

  src = builtins.filterSource (path: type: ! builtins.elem (baseNameOf path) [
    ".git" # leave out .git as it changes often in ways that do not affect the build
    "default.nix" # default.nix might change, but the only thing that matters is what it evaluates to, and nix takes care of that
    "result" # build result is irrelevant
    "build"
  ]) ./..;

  enableParallelInstalling = false;

  nativeBuildInputs = with pkgs; [ meson ninja pkg-config ];

  dontStrip = true;

  buildInputs = with pkgs; [
    SDL2
    libvorbis
    libogg
    libjpeg
    libpng
    freetype
    tinyxml
    openal
    freealut
    libepoxy
    curl
    readline
    miniupnpc
  ] ++ pkgs.lib.optional withEditor qt5.full
    ++ pkgs.lib.optionals withMape [ gtk3 gtksourceview ];

  preConfigure = ''
    sed s/REVGOESHERE/''${gitRef:0:12}/ > cmake/GitGetChangesetID.cmake <<EOF
    function(git_get_changeset_id VAR)
      set(\''${VAR} "REVGOESHERE" PARENT_SCOPE)
    endfunction()
    EOF
  '';

  mesonFlags = [
    (pkgs.lib.strings.mesonBool "editor" withEditor)
    (pkgs.lib.strings.mesonBool "mape" withMape)
  ];

  meta = with pkgs.lib; {
    description = "A free multiplayer action game about mining, settling and fast-paced melees";
    homepage = "http://www.openclonk.org/";
    license = with licenses; [
      isc cc-by-30
    ];
  };
}
