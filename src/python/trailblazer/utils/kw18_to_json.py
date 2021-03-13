# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import sys
import argparse

from trailblazer.utils.kw_utils import kw18_to_json

def main():
    # Setup input arguments
    parser = argparse.ArgumentParser(description='Convert a csv file to a new kw18 file');
    parser.add_argument("kw18_fname", type=str)
    parser.add_argument("json_fname", type=str)
    args = parser.parse_args()
    kw18_to_json(args.kw18_fname, args.json_fname)


if __name__ == '__main__':
    sys.exit(main())
