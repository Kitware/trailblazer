# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import ntpath
import pathlib
import argparse

from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.traffic.road_mapping import RoadMapping
from trailblazer.utils import kw_utils


def main(kw18_in=None, osm_fname=None, out_dir=None, overwrite=False):
    parser = argparse.ArgumentParser(
        description='Map a track set to a road network')
    parser.add_argument('--kw18_in',
                        help='kw18 file to read',
                        type=str,
                        default="./lair/ground_truth/full_frame_afrl/20091021_truth_rset0_frames0100-0611_train_converted_speed.kw18")
    parser.add_argument('--osm_fname',
                        help='OSM file containing the road network',
                        type=str,
                        default="./lair/map.osm")
    parser.add_argument('--out_dir',
                        help='Directory to write the mapping csv file',
                        type=str,
                        default=None)
    parser.add_argument('--plot_mapping',
                        help='If set, the mappings will be plotted and saved in <OUT_DIR>/figures',
                        default=False,
                        action='store_true')
    parser.add_argument('--zoom_plots',
                        help='Used in conjunction with "--plot_mapping", the plots will be zoomed to the track mappings rather than zooming to fit the entire road network',
                        default=True,
                        action='store_true')
    parser.add_argument('--overwrite', # Helpful if you just want to plot
                        help='Overwrite a preexisting mapping file if found, if false, mapping file will be left alone',
                        default=False,
                        action='store_false')

    args = parser.parse_args()
    # Only use args if main args are None
    if kw18_in is None:
        kw18_in = args.kw18_in
    if osm_fname is None:
        osm_fname = args.osm_fname
    if out_dir is None:
        out_dir = args.out_dir
    if out_dir is None:
        out_dir = os.path.dirname(kw18_in) + '/track_to_road_mapping/'
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    out_csv_name = ntpath.basename(kw18_in).replace("kw18", "csv")
    out_csv_path = os.path.join(out_dir, out_csv_name)

    if os.path.isfile(kw18_in):
        if not os.path.exists(out_csv_path):
            road_network = RoadNetwork.from_osm(osm_fname)
            track_set, mapping = convert(road_network, kw18_in, out_csv_path)

        if args.plot_mapping:
            if road_network is None:
                road_network = RoadNetwork.from_osm(osm_fname)
            if mapping is None:
                mapping = RoadMapping.from_csv(out_csv_path)
            if track_set is None:
                track_set = kw_utils.read_kw18(kw18_in)

            figures_outdir = os.path.join(out_dir, 'figures')
            pathlib.Path(figures_outdir).mkdir(parents=True, exist_ok=True)

            mapping.visualize_full_paths(figures_outdir,
                                         road_network,
                                         track_set,
                                         zoom_to_track=args.zoom_plots)
    else:
        found_kw18s = {}
        road_network = RoadNetwork.from_osm(osm_fname)
        for dirpath, dirs, files in os.walk(kw18_in):
            if len(files) == 0 or all('.kw18' not in file for file in files):
                continue  # No kw18 here, move on...
            # Find all the kw18 files to run on
            for filename in [f for f in files]:
                if filename == "linked_tracks.kw18":
                    kw18 = os.path.join(dirpath, filename)
                    found_kw18s[kw18] = dirpath
        print("Found " + str(len(found_kw18s)) + " kw18 files")

        for kw18, path in found_kw18s.items():
            out_csv_name = ntpath.basename(kw18).replace("kw18", "csv")
            out_csv_path = os.path.join(dirpath, out_csv_name)
            convert(road_network,kw18,out_csv_path)


def convert(road_network, kw18_in, out_csv_path):
    track_set = kw_utils.read_kw18(kw18_in)

    mapping = RoadMapping.map_track_set_to_roads(track_set, road_network)
    mapping.save_csv(out_csv_path)

    return track_set,mapping


if __name__ == '__main__':
    main()
