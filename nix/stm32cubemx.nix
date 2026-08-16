{
  makeWrapper,
  symlinkJoin,
  fdupes,
  buildFHSEnv,
  fetchzip,
  icoutils,
  imagemagick,
  jdk21,
  lib,
  makeDesktopItem,
  stdenvNoCC,
}:

let
  iconame = "STM32CubeMX";
  package = stdenvNoCC.mkDerivation rec {
    pname = "stm32cubemx";
    version = "6.15.0";

    src = fetchzip {
      url = "https://sw-center.st.com/packs/resource/library/stm32cube_mx_v${
        builtins.replaceStrings [ "." ] [ "" ] version
      }-lin.zip";
      hash = "sha256-50P+/uvNH3NN1UN+T3RxGgR8QYBIgBDA56mAEU4BipI=";
      stripRoot = false;
    };

    nativeBuildInputs = [
      fdupes
      icoutils
      imagemagick
    ];
    desktopItem = makeDesktopItem {
      name = "STM32CubeMX";
      exec = "stm32cubemx";
      desktopName = "STM32CubeMX";
      categories = [ "Development" ];
      icon = "stm32cubemx";
      comment = meta.description;
      terminal = false;
      startupNotify = false;
      mimeTypes = [
        "x-scheme-handler/sgnl"
        "x-scheme-handler/signalcaptcha"
      ];
    };

    buildCommand = ''
      mkdir -p $out/{bin,opt/STM32CubeMX,share/applications}

      cp -r $src/MX/. $out/opt/STM32CubeMX/
      chmod +rx $out/opt/STM32CubeMX/STM32CubeMX

      cat << EOF > $out/bin/${pname}
      #!${stdenvNoCC.shell}
      updater_xml="\$HOME/.stm32cubemx/thirdparties/db/updaterThirdParties.xml"
      if [ -e "\$updater_xml" ] && [ ! -w "\$updater_xml" ]; then
        echo "Warning: Unwritable \$updater_xml prevents CubeMX software packages from working correctly. Fixing that."
        (set -x; chmod u+w "\$updater_xml")
      fi
      ${jdk21}/bin/java -jar $out/opt/STM32CubeMX/STM32CubeMX "\$@"
      EOF
      chmod +x $out/bin/${pname}

      fdupes -dN . > /dev/null
      ls
      for size in 16 24 32 48 64 128 256; do
        mkdir -pv $out/share/icons/hicolor/"$size"x"$size"/apps
        if [ $size -eq 256 ]; then
          cp $out/opt/STM32CubeMX/help/${iconame}.png \
            $out/share/icons/hicolor/"$size"x"$size"/apps/${pname}.png
        else
          magick $out/opt/STM32CubeMX/help/${iconame}.png -resize "$size"x"$size" \
            $out/share/icons/hicolor/"$size"x"$size"/apps/${pname}.png
        fi
      done;

      cp ${desktopItem}/share/applications/*.desktop $out/share/applications
      if ! grep -q StartupWMClass= "$out"/share/applications/*.desktop; then
          chmod +w "$out"/share/applications/*.desktop
          echo "StartupWMClass=com-st-microxplorer-maingui-STM32CubeMX" >> "$out"/share/applications/*.desktop
      else
          echo "error: upstream already provides StartupWMClass= in desktop file -- please update package expr" >&2
          exit 1
      fi
    '';

    meta = {
      description = "Graphical tool for configuring STM32 microcontrollers and microprocessors";
      longDescription = ''
        A graphical tool that allows a very easy configuration of STM32
        microcontrollers and microprocessors, as well as the generation of the
        corresponding initialization C code for the Arm® Cortex®-M core or a
        partial Linux® Device Tree for Arm® Cortex®-A core), through a
        step-by-step process.
      '';
      homepage = "https://www.st.com/en/development-tools/stm32cubemx.html";
      sourceProvenance = with lib.sourceTypes; [ binaryBytecode ];
      license = lib.licenses.unfree;
      maintainers = with lib.maintainers; [
        angaz
        wucke13
      ];
      platforms = [ "x86_64-linux" ];
    };
  };
  # CubeMX hardcodes $HOME/.stm32cubemx (device database, ~1GB) and
  # $HOME/STM32Cube/Repository (firmware packs, several hundred MB more).
  # Redirecting HOME does not move them: the JDK resolves user.home from the
  # password database, not the environment. Bind-mounting inside the existing
  # bubblewrap sandbox does work, and keeps the user's real home clean.
  #
  # extraBwrapArgs is interpolated into a shell script, so these expand at
  # runtime. The source directories are created by the wrapper below, because
  # bwrap refuses to bind a path that does not exist.
  stateDir = ''"''${XDG_DATA_HOME:-$HOME/.local/share}/stm32cubemx"'';

  fhs = buildFHSEnv {
  inherit (package) pname version meta;
  runScript = "${package.outPath}/bin/stm32cubemx";
  extraBwrapArgs = [
    ''--bind ${stateDir}/dot-stm32cubemx "$HOME/.stm32cubemx"''
    ''--bind ${stateDir}/STM32Cube "$HOME/STM32Cube"''
  ];
  extraInstallCommands = ''
    mkdir -p $out/share/{applications,icons}
    ln -sf ${package.outPath}/share/applications/* $out/share/applications/
    ln -sf ${package.outPath}/share/icons/* $out/share/icons/
  '';
  targetPkgs =
    pkgs: with pkgs; [
      alsa-lib
      at-spi2-atk
      cairo
      cups
      dbus
      expat
      glib
      gtk3
      libdrm
      libGL
      libudev0-shim
      libxkbcommon
      libgbm
      nspr
      nss
      pango
      libx11
      libxcb
      libxcomposite
      libxdamage
      libxext
      libxfixes
      libxrandr
      libgcrypt
      openssl
      udev
    ];
  };
in
# Thin wrapper that creates the state directories before bwrap tries to bind
# them, then hands over to the sandboxed CubeMX. symlinkJoin keeps the desktop
# entry and icons the FHS env installs.
symlinkJoin {
  inherit (package) pname version meta;
  name = "${package.pname}-${package.version}";
  paths = [ fhs ];
  nativeBuildInputs = [ makeWrapper ];
  # Single-quoted so the build shell leaves $HOME alone; it must expand when
  # the wrapper runs, not while nix is building it (where HOME is
  # /homeless-shelter).
  postBuild = ''
    rm -f $out/bin/stm32cubemx
    makeWrapper ${fhs}/bin/stm32cubemx $out/bin/stm32cubemx \
      --run 'state="''${XDG_DATA_HOME:-$HOME/.local/share}/stm32cubemx"; mkdir -p "$state/dot-stm32cubemx" "$state/STM32Cube"'
  '';
}
