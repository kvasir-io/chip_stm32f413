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

root := source_directory()
self := source_file()   # not justfile(): that names the ROOT justfile when loaded as a module

# What this package is, and where to build it from
default:
    @echo "chip_stm32f413 - a CMake include()-fragment, consumed through CHIP_ROOT."
    @echo ""
    @echo "  Nothing builds here. To exercise this package:"
    @echo "      cd <a firmware on this chip> && just chip_root={{ root }} build-all"
    @echo ""
    @just --list --unsorted

# `git submodule update` discards uncommitted work inside a populated submodule,
# and this tree has some, so only missing ones are touched.
# Initialise any submodule that is not checked out yet, core/'s own included
submodules:
    #!/usr/bin/env bash
    set -euo pipefail
    root="{{ root }}"
    # Walk .gitmodules (see the top): a submodule is missing when its path is absent or
    # empty. This repository first, so a fresh core/ is there to walk next.
    initialised=0
    for repo in "$root" "$root/core"; do
        [ -f "$repo/.gitmodules" ] || continue
        missing=()
        while read -r _ path; do
            full="$repo/$path"
            # a symlinked path is deliberate - never re-initialise over it
            if [ -L "$full" ]; then
                continue
            fi
            if [ ! -d "$full" ] || [ -z "$(ls -A "$full" 2>/dev/null)" ]; then
                missing+=("$path")
            fi
        done < <(git -C "$repo" config -f "$repo/.gitmodules" --get-regexp '^submodule\..*\.path$')
        [ ${#missing[@]} -ne 0 ] || continue
        printf 'initialising in %s: %s\n' "$repo" "${missing[*]}"
        git -C "$repo" submodule update --init --recursive -- "${missing[@]}"
        initialised=1
    done
    [ $initialised -eq 1 ] || echo "all submodules present"

# No .clang-format is checked in, so this follows ~/.clang-format - the same
# config svd_converter formats its generated headers with.
# List the sources clang-format would rewrite
format-check:
    #!/usr/bin/env bash
    set -euo pipefail
    # pioasm is vendored upstream code - read as documentation, never rewritten.
    mapfile -t files < <(find "{{ root }}/src" "{{ root }}/core/src" \
        -path '*/pioasm/*' -prune -o \
        \( -name '*.hpp' -o -name '*.cpp' \) -print 2>/dev/null | sort)
    [ ${#files[@]} -ne 0 ] || { echo "no sources found"; exit 0; }
    dirty=()
    for f in "${files[@]}"; do
        clang-format --output-replacements-xml "$f" | grep -q '<replacement ' && dirty+=("$f")
    done
    if [ ${#dirty[@]} -eq 0 ]; then
        echo "${#files[@]} files, all formatted"
        exit 0
    fi
    printf 'would reformat (%d of %d):\n' "${#dirty[@]}" "${#files[@]}"
    printf '  %s\n' "${dirty[@]#{{ root }}/}"
    exit 1

# Rewrite those sources in place. Nothing else calls this - the diff is yours.
format:
    #!/usr/bin/env bash
    set -euo pipefail
    # pioasm is vendored upstream code - read as documentation, never rewritten.
    mapfile -t files < <(find "{{ root }}/src" "{{ root }}/core/src" \
        -path '*/pioasm/*' -prune -o \
        \( -name '*.hpp' -o -name '*.cpp' \) -print 2>/dev/null | sort)
    [ ${#files[@]} -ne 0 ] || { echo "no sources found"; exit 0; }
    clang-format -i "${files[@]}"
    echo "formatted ${#files[@]} files"

# Build the container image CI publishes as kvasirio/stm32f413:latest
docker tag="kvasirio/stm32f413:latest":
    docker build -f "{{ root }}/scripts/Dockerfile.stm32f413" -t "{{ tag }}" "{{ root }}"
