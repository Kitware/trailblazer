import os
import pathlib
import argparse
from trailblazer.utils import kw_utils


def main(kw18_in=None, osm_fname=None, out_dir=None, rate_Hz=None):
    parser = argparse.ArgumentParser(description='Downsample a track set to a given rate')
    parser.add_argument('--kw18_in',
                        help='kw18 file to read',
                        type=str,
                        default="./net_quantico/routes/traffic0/high_density/run0/sumo.kw18")
    parser.add_argument('--osm_fname',
                        help='OSM file to use to generate vpView image\n'
                             'If not provided the image will be generated from the kw18_in',
                        type=str,
                        default="./net_quantico/osm_bbox.osm.xml")
    parser.add_argument('--out_dir',
                        help='Directory to write the new kw18 and associated vpView project.\n'
                             'If not provided, a new directory named vpView will be created in the input file dir',
                        type=str,
                        default=None)
    parser.add_argument('--rate_Hz',
                        help='Rate, in Hz, at which to sample track frequency to.',
                        type=int,
                        default=1)
    args = parser.parse_args()
    # Only use args if main args are None
    if kw18_in is None:
        kw18_in = args.kw18_in
    if osm_fname is None:
        osm_fname = args.osm_fname
    if out_dir is None:
        out_dir = args.out_dir
    if rate_Hz is None:
        rate_Hz = args.rate_Hz

    if out_dir is None:
        out_dir = os.path.dirname(kw18_in) + '/vpView_downsample/'
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    track_set = kw_utils.read_kw18(kw18_in)
    ds_track_set = kw_utils.downsample_track_set(track_set, rate_Hz)
    kw_utils.create_vpView(ds_track_set, out_dir, osm_fname)

if __name__ == '__main__':
    main()