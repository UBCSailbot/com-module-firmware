{
  description = "UBCSailbot COM module firmware";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems =
        f:
        nixpkgs.lib.genAttrs systems (
          system:
          f (
            import nixpkgs {
              inherit system;
              # STM32CubeMX is distributed under ST's own licence.
              config.allowUnfree = true;
            }
          )
        );
    in
    {
      # CubeMX pinned to the version every .ioc in this repo is generated with.
      # Opening a project in a different CubeMX raises a modal migrate dialog
      # that blocks even headless runs, so the version is part of the build
      # environment rather than something each developer installs by hand.
      # Keep in step with EXPECTED_CUBEMX in tools/check_ioc_sync.py.
      packages = forAllSystems (pkgs: {
        stm32cubemx = pkgs.callPackage ./nix/stm32cubemx.nix { };
        default = self.packages.${pkgs.system}.stm32cubemx;
      });

      devShells = forAllSystems (pkgs: {
        # What CI uses. Deliberately excludes CubeMX and the cross toolchain:
        # neither the host unit tests nor the .ioc check need them, and CubeMX
        # alone is a ~1GB unfree download that would dominate every run.
        ci = pkgs.mkShell {
          # firmware/tests builds at -O0 with -Werror. nixpkgs' cc-wrapper adds
          # -D_FORTIFY_SOURCE by default, and glibc then warns that fortify
          # needs optimisation, which -Werror turns into a hard error. The
          # default differs between a NixOS host and a plain runner, so set it
          # explicitly rather than relying on the ambient environment.
          hardeningDisable = [
            "fortify"
            "fortify3"
          ];

          packages = [
            pkgs.gcc
            pkgs.gnumake
            pkgs.python3
          ];
        };

        default = pkgs.mkShell {
          # Same reason as the ci shell: firmware/tests is -O0 -Werror.
          hardeningDisable = [
            "fortify"
            "fortify3"
          ];

          packages = [
            self.packages.${pkgs.system}.stm32cubemx

            # Cross toolchain. Note this is not the same build as ST's bundled
            # "GNU Tools for STM32", so binaries it produces will not be
            # byte-identical to a CubeIDE build of the same source.
            pkgs.gcc-arm-embedded

            # Flashing and on-target debug.
            pkgs.openocd
            pkgs.stlink

            # Host unit tests (firmware/tests) and tools/check_ioc_sync.py.
            pkgs.gcc
            pkgs.gnumake
            pkgs.python3
          ];

          shellHook = ''
            echo "com-module-firmware devshell"
            echo "  stm32cubemx : $(stm32cubemx --version 2>/dev/null || echo 6.15.0)"
            echo "  arm gcc     : $(arm-none-eabi-gcc -dumpversion 2>/dev/null)"
            echo
            echo "  make -C firmware/tests test   run host unit tests"
            echo "  tools/check_ioc_sync.py .     check .ioc against generated code"
          '';
        };
      });
    };
}
