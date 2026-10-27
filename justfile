# Upkeep helpers for the stm32f413 chip package: submodules, formatting, the container image.
#
# Nothing builds here: cmake/chip.cmake is an include()-fragment the Kvasir SDK pulls in through CHIP_ROOT, and the
# peripheral headers are generated from chip.svd into the consumer's build tree. To check the package, build a user:
#
#     cd <a firmware on this chip> && just chip_root=$PWD/../chip_stm32f413 build-all
#
# core/ (and core/'s src/cortex_common) are normally symlinks made by the workspace's link step; `git
# submodule status` refuses a symlinked path, which is why `just submodules` reads .gitmodules.

set shell := ["bash", "-euo", "pipefail", "-c"]

# chip.just must never define root: source_directory() there is Kvasir_SDK/just, not this package
root := source_directory()
chip := "stm32f413"
# the --build-arg this repository's stm32f413-docker-build.yml passes to scripts/Dockerfile.chip
docker_args := "--build-arg WITH_KVASIR_DEVICES=0"
# the Kvasir_SDK checkout next to this one: the shared recipes, the Dockerfile and the lint scanner come from it
sdk := root / ".." / "Kvasir_SDK"

# What this package is, and where to build it from
default:
    @echo "chip_{{ chip }} - a CMake include()-fragment, consumed through CHIP_ROOT."
    @echo ""
    @echo "  Nothing builds here. To exercise this package:"
    @echo "      cd <a firmware on this chip> && just chip_root={{ root }} build-all"
    @[ -f "{{ sdk }}/just/chip.just" ] || echo "  (no {{ sdk }}/just/chip.just: check out Kvasir_SDK next to this repo for the other recipes)"
    @echo ""
    @just --justfile "{{ source_file() }}" --list --unsorted

# submodules, format-check, format, docker, lint
import? '../Kvasir_SDK/just/chip.just'
