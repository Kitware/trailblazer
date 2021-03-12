import os
import yaml
import pathlib
import argparse

import trailblazer.traffic.map_tracks_to_roads as mtr
from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.feature import features
from trailblazer.feature import analyze

# Compare multiple traffic sets
# Set the working directory to your root data directory

def main(src_dir=None, yaml_fname=None, base_layer=None, osm_fname=None, out_dir=None, overwrite=False):
    parser = argparse.ArgumentParser(
    description='Compare a set of kw18 traffic files')
    parser.add_argument('--src_dir',
                      help="Directory of kw18s to compare",
                      type=str,
                      default="./analysis")
    parser.add_argument('--yaml_fname',
                      help='YAML file for feature selection',
                      type=str,
                      default="./analysis/analysis.yaml")
    parser.add_argument('--base_layer',
                        help='Base layer for the road network',
                        type=str,
                        default="./analysis/base_layer.tif")
    parser.add_argument('--osm_fname',
                      help='OSM file containing the road network',
                      type=str,
                      default="./net_quantico/osm_bbox.osm.xml")
    parser.add_argument('--out_dir',
                      help='Directory to write the mapping csv file',
                      type=str,
                      default=None)
    parser.add_argument('--overwrite',
                        help='Overwrite any intermediate files',
                        default=False,
                        action='store_false')

    args = parser.parse_args()
    # Only use args if main args are None
    if src_dir is None:
        src_dir = args.src_dir
    if yaml_fname is None:
        yaml_fname = args.yaml_fname
    if base_layer is None:
        base_layer = args.base_layer
    if osm_fname is None:
        osm_fname = args.osm_fname
    if out_dir is None:
        out_dir = args.src_dir
    if not os.path.isdir(out_dir):
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    if overwrite is None:
        overwrite = args.src_dir


    if yaml_fname is None:
        print("Must provide a yaml feature configuration file")
        return

    with open(yaml_fname) as f:
        cfg, = yaml.safe_load_all(f)

    found_kw18s = []
    for dirpath, dirs, files in os.walk(src_dir):
        if len(files) == 0 or all('.kw18' not in file for file in files):
          continue  # No kw18 here, move on...
        # Find all the kw18 files to run on
        for filename in [f for f in files]:
            if filename.endswith('.kw18'):
                kw18 = os.path.join(dirpath, filename)
                found_kw18s.append(kw18)
    print("Found " + str(len(found_kw18s)) + " kw18 files")

    if not os.path.exists(base_layer):
        RoadNetwork.from_osm(osm_fname)._base_layer.save_geotiff(base_layer)

    feature_files = []
    for kw18 in found_kw18s:
        print("Processing "+kw18)
        # Map the tracks to the road network, if not already done
        mtr.main(kw18_in=kw18, osm_fname=osm_fname, out_dir=src_dir, overwrite=overwrite)
        # Write out features for traffic set
        cfg['input']['feature_input']['track_set'] = kw18
        cfg['input']['feature_input']['road_mapping'] = kw18.replace("kw18","csv")
        cfg['input']['feature_input']['road_network'] = osm_fname
        cfg['input']['feature_input']['base_layer'] = base_layer
        feature_files.append(kw18.replace("kw18","npy"))
        features.write_features(cfg, feature_files[-1], overwrite)
        print("Finished Processing " + kw18)

    analyze.compare_features(feature_files)

if __name__ == '__main__':
    main()


