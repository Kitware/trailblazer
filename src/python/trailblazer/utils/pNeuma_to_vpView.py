# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import pathlib
import argparse
from trailblazer.utils import kw_utils


def main():
    parser = argparse.ArgumentParser(description='Create a vpView project from a kw18 file')
    parser.add_argument('--csv_in',
                        help='pNeuma csv file to read',
                        type=str,
                        default="./pNeuma/20181024_d1_0900_0930.csv")
    parser.add_argument('--out_dir',
                        help='directory to write vpView project files.',
                        type=str,
                        default=None)
    parser.add_argument('--osm_file',
                        help='Generate base layer image from osm rather than track history.',
                        type=str,
                        default="./pNeuma/location_1.osm")
    parser.add_argument('--base_layer_fname',
                        help='Baselayer image to use',
                        type=str,
                        default=None)
    args = parser.parse_args()

    out_dir = args.out_dir
    if out_dir is None:
        out_dir = os.path.dirname(args.csv_in) + '/vpView/'
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    track_set = kw_utils.read_pNeuma(args.csv_in)
    new_tracks = kw_utils.downsample_track_set(track_set, 1.0)
    kw_utils.create_vpView(new_tracks, out_dir, args.osm_file, args.base_layer_fname)

if __name__ == '__main__':
    main()