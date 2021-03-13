# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import pathlib
import argparse
from trailblazer.utils import kw_utils


def main():
    parser = argparse.ArgumentParser(description='Create a vpView project from a kw18 file')
    parser.add_argument('--kw18_in',
                        help='kw18 file to read',
                        type=str,
                        default="./net_quantico/routes/traffic0/high_density/run0/sumo.kw18")
    parser.add_argument('--out_dir',
                        help='directory to write vpView project files.',
                        type=str,
                        default=None)
    parser.add_argument('--osm_file',
                        help='Generate base layer image from osm rather than track history.',
                        type=str,
                        default="./net_quantico/osm_bbox.osm.xml")
    parser.add_argument('--base_layer_fname',
                        help='Baselayer image to use',
                        type=str,
                        default=None)
    args = parser.parse_args()

    out_dir = args.out_dir
    if out_dir is None:
        out_dir = os.path.dirname(args.kw18_in) + '/vpView/'
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    track_set = kw_utils.read_kw18(args.kw18_in)
    kw_utils.create_vpView(track_set, out_dir, args.osm_file, args.base_layer_fname)

if __name__ == '__main__':
    main()