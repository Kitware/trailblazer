import os
import pathlib
import argparse

from trailblazer.traffic.base_layer import BaseLayer
from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.utils import kw_utils


def main(kw18_in=None, osm_fname=None, out_dir=None, gsd=None):
    parser = argparse.ArgumentParser(description='Create a baselayer given a gsd')
    parser.add_argument('--kw18_in',
                        help='kw18 file to read',
                        type=str,
                        default="./data/net_quantico/routes/traffic0/high_density/run0/sumo.kw18")
    parser.add_argument('--osm_fname',
                        help='OSM file to use to generate vpView image\n'
                             'If not provided the image will be generated from the kw18_in',
                        type=str,
                        default="./data/net_quantico/osm_bbox.osm.xml")
    parser.add_argument('--out_dir',
                        help='Directory to write the new kw18 and associated vpView project.\n'
                             'If not provided, a new directory named vpView will be created in the input file dir',
                        type=str,
                        default=None)
    parser.add_argument('--gsd',
                        help='Ground sample distance used for resampling',
                        type=float,
                        default=5.0)
    args = parser.parse_args()
    # Only use args if main args are None
    if kw18_in is None:
        kw18_in = args.kw18_in
    if osm_fname is None:
        osm_fname = args.osm_fname
    if out_dir is None:
        out_dir = args.out_dir
    if gsd is None:
        gsd = args.gsd

    if out_dir is None:
        out_dir = os.path.dirname(kw18_in) + '/vpView_gsd_degrade_{}/'.format(gsd)
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    road_network = RoadNetwork.from_osm(osm_fname)
    base_layer = BaseLayer.from_road_network(road_network, gsd=gsd)

    track_set = kw_utils.read_kw18(kw18_in)

    degraded_track_set = kw_utils.compute_gsd_world_coordinates(track_set, base_layer)
    kw_utils.write_vpView_files(degraded_track_set, out_dir, base_layer)


if __name__ == '__main__':
    main()
