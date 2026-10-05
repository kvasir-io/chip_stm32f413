#!/usr/bin/env python3
"""Compiles one test file the way a firmware tree compiles its main.cpp, and passes when it compiles clean.

Usage: firmware_check.py <compile_commands.json> <main suffix> <file>

The chip headers need the generated registers and a board, so they cannot be built on the host. This takes the
compile command of the entry whose file ends in <main suffix> (the first variant found), compiles <file> with it
(-fsyntax-only, nothing written into the tree) and fails on any diagnostic. The test files are static_asserts.
"""

import json
import shlex
import subprocess
import sys


def main():
    db, suffix, source = sys.argv[1], sys.argv[2], sys.argv[3]
    with open(db) as fh:
        entries = json.load(fh)
    entry = next((e for e in entries if e["file"].endswith(suffix)), None)
    if entry is None:
        print(f"no entry for {suffix} in {db}")
        return 1
    args = entry["arguments"] if "arguments" in entry else shlex.split(
        entry["command"])
    if "ccache" in args[0]:
        args = args[1:]
    out = []
    skip = False
    for a in args:
        if skip:
            skip = False
            continue
        if a in ("-o", "-MF", "-MT", "-MQ"):
            skip = True
            continue
        if a in ("-c", "-MD", "-MMD") or a == entry["file"] or a.endswith(suffix):
            continue
        out.append(a)
    # a test file has no main(): StartUp.hpp's `startup` is then unused, which the firmware never sees
    cmd = out + ["-fsyntax-only", "-Wno-unneeded-internal-declaration", source]
    r = subprocess.run(
        cmd, cwd=entry["directory"], capture_output=True, text=True)
    text = r.stdout + r.stderr
    if r.returncode != 0 or "warning:" in text:
        print(text[-4000:])
        print("FAIL: expected a clean compile")
        return 1
    print("ok: compiles without a warning")
    return 0


if __name__ == "__main__":
    sys.exit(main())
