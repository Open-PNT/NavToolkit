#!/usr/bin/env python


import os
import sys

if len(sys.argv) < 3:
    print(
        "Usage: find_sources.py src/dir cpp c # Find .cpp and .c files in "
        + "src/dir"
    )
    exit(1)

sep = " "
searchpath = sys.argv[1]
extensions = sys.argv[2:]

outarr = []

for p, _, farr in os.walk(searchpath):
    for f in farr:
        for ext in extensions:
            if f.endswith("." + ext):
                outarr.append(os.path.join(p, f))

if len(outarr) > 0:
    print(sep.join(outarr))
