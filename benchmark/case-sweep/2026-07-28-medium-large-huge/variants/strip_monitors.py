#!/usr/bin/env python3
"""Write a monitor-free copy of a PhasorDynamics case.

Removes every `mon` key and empties `monitors`, so a run does no file I/O and
the measurement contains no formatting or write cost. Case files are large and
are regenerated rather than stored here.

    python3 strip_monitors.py <src.case.json> <dst.case.json>
"""
import json
import sys


def strip(node):
    if isinstance(node, dict):
        node.pop("mon", None)
        for value in node.values():
            strip(value)
    elif isinstance(node, list):
        for value in node:
            strip(value)


def main():
    case = json.load(open(sys.argv[1]))
    strip(case)
    case["monitors"] = []
    json.dump(case, open(sys.argv[2], "w"))


if __name__ == "__main__":
    main()
