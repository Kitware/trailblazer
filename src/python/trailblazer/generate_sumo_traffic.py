import argparse
import json
import os
import sumolib

from trailblazer.sumo.add_traffic import RouteBuilder
from trailblazer.sumo.start_sumo import SUMO_exec
from trailblazer.sumo.fcd_utils import FCD
from trailblazer.traffic.base_layer import BaseLayer
from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.utils import pixel_utils as px
from trailblazer.utils.path_utils import setup_paths
# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

from trailblazer.utils import kw_utils

# Class to generate a random traffic set using sumo
# Set the working directory to your root data directory

def main():
    parser = argparse.ArgumentParser(description='Run all SUMO components')
    parser.add_argument('--start_run',
                        help='Number to start counting runs at',
                        type=int,
                        default=0)
    parser.add_argument('--num_runs',
                        help='Number of runs to execute.',
                        type=int,
                        default=1)
    parser.add_argument('--net_directory',
                        help='Directory path containing SUMO network.',
                        type=str,
                        default=None)
    parser.add_argument('--output_directory',
                        help='Directory path to generate all data runs in.',
                        type=str,
                        default='./routes')
    parser.add_argument('--timestep',
                        help='Time in between each simulation timestep.',
                        type=float,
                        default=1)
    parser.add_argument('--density',
                        help="Generate 'low', 'medium', 'high', 'all', or 'custom' traffic density",
                        type=str,
                        default='high')
    parser.add_argument('--custom_density',
                        help='JSON object literal mapping vehicle types to densities',
                        type=json.loads)
    parser.add_argument('--sim_length',
                        help='Time of simulation in seconds',
                        type=int,
                        default=800)
    parser.add_argument('--start_record',
                        help='Time to start recording data to kw18',
                        type=int,
                        default=200)


    args = parser.parse_args()

    if (args.density == 'custom') == (args.custom_density is None):
        print("Custom density must be provided exactly when the density is specified as 'custom'")
        return

    if args.net_directory is None:
        print("Specify the SUMO net directory to use")
        return

    full_network_dir, out_dir = setup_paths(args.net_directory, args.output_directory)
    net_xml = full_network_dir + "/osm.net.xml"
    sumo_net = sumolib.net.readNet(net_xml)
    # NOTE: Truth segment files generated below are of 'Simplified' edges!!!
    #       Using the edges from SUMO gives us 'Simplified' edges
    #       Meaning, if there is no intersection, osm edges are combined
    #       So a curved road made of small edges, will actually be one edge
    if not sumo_net:
        print('Could not open network ' + net_xml)
        return ()
    osm_file = args.net_directory + "/osm_bbox.osm.xml"
    road_network = RoadNetwork.from_osm(osm_file)
    road_networkG = road_network.network
    # Write out the base layer to use for vpView projects
    base_layer_fname = out_dir + '/base_layer.tif'
    if not os.path.isfile(base_layer_fname):
        base_layer = BaseLayer.from_road_network(road_network)
        base_layer.save_geotiff(base_layer_fname)

    vehicle_densities = (
        RouteBuilder.vehicle_densities if args.density == 'all' else
        {'custom': args.custom_density} if args.density == 'custom' else
        {args.density: RouteBuilder.vehicle_densities[args.density]}
    )

    routes = RouteBuilder()
    starter = SUMO_exec()

    # Generate a run with only traffic and no anomaly
    for density, densities in vehicle_densities.items():
        for i in range(args.start_run, args.start_run+args.num_runs):
            sub_dir = '/traffic0/' + density + '_density/run' + str(i) + '/'
            print('Simulating ' + sub_dir)
            # Setup and run SUMO
            network_dir, route_run_dir = setup_paths(args.net_directory, args.output_directory + sub_dir)
            routes.build_routes_from_densities(sumo_net, net_xml, route_run_dir, densities, args.sim_length)
            starter.start_sim(network_dir, route_run_dir, args.timestep, args.sim_length)
            track_set, event_set = FCD.convert2kw18(sumo_net, route_run_dir, args.start_record, args.sim_length)

            # Write out our kw files
            print("Writing kw18...")
            kw_utils.write_kw18(track_set, route_run_dir + "/sumo.kw18")
            px.world_to_image_coordinates(base_layer, track_set)
            kw_utils.write_vpView_files(track_set, route_run_dir + "/vpView", base_layer_fname)

            # Write timing to a file
            text_file = open(route_run_dir + "timing.txt", "w+")
            text_file.write("Simulation Length(s): %s\n" % (args.sim_length - args.start_record))
            text_file.close()


if __name__ == "__main__":
    main()
