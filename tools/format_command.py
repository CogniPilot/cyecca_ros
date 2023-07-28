#!/usr/bin/env python3
import subprocess
import os
import sys
import re
import argparse
from pathlib import Path

root_dir = Path(os.path.dirname(os.path.abspath(sys.argv[0]))).parent

parser = argparse.ArgumentParser('format')
parser.add_argument('-c', '--check', action='store_true')
args = parser.parse_args()

source_paths = [
    root_dir / 'src'
]  # type: List[Path]

regex = re.compile('.*\.(c|cpp|hpp)$')
format_ok = True

for path in source_paths:
    for file in path.rglob("*"):
        if regex.match(str(file)):
            if args.check:
                cmd_str = ['clang-format-14', '--dry-run', '--Werror', '-style', 'WebKit', f'{file}']
            else:
                cmd_str = ['clang-format-14', '-i', '-style', 'WebKit', f'{file}']
            try:
                res = subprocess.run(cmd_str, capture_output=False, check=True)
            except Exception as e:
                print(e)
                format_ok = False
if not format_ok:
    exit(1)
