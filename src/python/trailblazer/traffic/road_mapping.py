from itertools import chain
import os
import pathlib
import csv

import numpy as np
from leuvenmapmatching.matcher.distance import DistanceMatcher
from leuvenmapmatching.map.inmem import InMemMap
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import networkx as nx

from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.traffic.base_layer import BaseLayer
from trailblazer.utils import kw_utils


class RoadMapping(object):
    __slots__ = ['full_paths',
                 'simplified_paths']

    def __init__(self):
        """Initialize an empty RoadMapping

        Both full_paths and simplified_paths are dictionaries of the
        form: { track_id: [ path_edges, times ] }, where path_edges[i]
        is the edge to which the track has been mapped at times[i]

        The edges in simplified_paths are from the simplified road
        network graph
        """
        self.full_paths = {}
        self.simplified_paths = {}

    @classmethod
    def build_map_matching_graph(cls, road_network):
        """Return leuvenmapmatching.BaseMap used for map matching.

        """
        # Create the Leuven.MapMatching graph
        G = road_network.network
        map_con = InMemMap('road_network', use_latlon=False,
                           use_rtree=True, index_edges=True,
                           crs_lonlat=None, crs_xy=None, graph=None)
        for key in G.nodes:
            node = G.nodes[key]
            try:
                map_con.add_node(key, (node['easting'], node['northing']))
            except KeyError:
                raise Exception('Must call '
                                '\'populate_meters_from_lon_lat\' first '
                                'to define the Cartesian coordinates '
                                'associated with the latitude and '
                                'longitude coordinates.')

        for edge in G.edges:
            map_con.add_edge(*edge)

        return map_con

    @classmethod
    def _unique_paths(path, times):
        unique_path = [path[0]]
        unique_times = [times[0]]
        for i in range(1, len(path)):
            if path[i-1] != path[i]:
                unique_path.append(path[i])
                unique_times.append(times[i])

        return unique_path, unique_times

    @classmethod
    def map_track_to_roads(cls,
                           track_positions,
                           map_matching_graph,
                           stepsize=5,
                           obs_noise=5,
                           obs_noise_ne=5,
                           max_dist_init=1000,
                           max_dist=5,
                           min_prob_norm=0.5,
                           non_emitting_states=False,
                           non_emitting_length_factor=0.75,
                           max_lattice_width=10,
                           dist_noise=5,
                           dist_noise_ne=5,
                           restrained_ne=True,
                           avoid_goingback=True):
        """Map a series of track_positions to a road network using a map_matching_graph

        :param track_positions: a numpy array of [x, y, timestamp]
        where x and y are assumed to be in meters

        :param map_matching_graph: The map matching graph for a road network

        :return: An array of road edges and corresponding timestamps.
        When a road mapping cannot be made for a given track position,
        a 'None' is inserted
        """
        track_arc_lengths_m = np.hstack(
            [0, np.cumsum(
                np.sqrt(np.sum(np.diff(track_positions[:, :2], axis=0)**2,
                               1)))])
        total_track_length_m = track_arc_lengths_m[-1]
        if total_track_length_m == 0.0:
            print(f"Total track length must be > 0.0, skipping!")
            return [], []
        elif total_track_length_m > 100000:
            print(f"Total track length is abnormally large "
                  f"({total_track_length_m} meters), skipping!")
            return [], []

        # We need to remove positions with duplicate x, y to be able
        # to interpolate (always keeping the latter of the two; we
        # never throw away the last point)
        ind = np.hstack([np.diff(track_arc_lengths_m), 1]) != 0

        interp_f = interp1d(track_arc_lengths_m[ind],
                            track_positions[ind],
                            axis=0)

        steps = np.linspace(0,
                            total_track_length_m,
                            int(np.ceil(total_track_length_m/stepsize)) + 1)

        interp_positions = interp_f(steps)

        node_path = []
        k = 0
        matcher = DistanceMatcher(map_matching_graph,
                                      obs_noise=obs_noise,
                                      obs_noise_ne=obs_noise_ne,
                                      max_dist_init=max_dist_init,
                                      max_dist=max_dist,
                                      min_prob_norm=min_prob_norm,
                                      non_emitting_states=non_emitting_states,
                                      non_emitting_length_factor=non_emitting_length_factor,
                                      max_lattice_width=max_lattice_width,
                                      dist_noise=dist_noise,
                                      dist_noise_ne=dist_noise_ne,
                                      restrained_ne=restrained_ne,
                                      avoid_goingback=avoid_goingback)
        while k < len(interp_positions):


            node_pathi, ind = matcher.match(
                interp_positions[k:, :2])
            if len(node_pathi) == 0:
                node_path.append(None)
                k += 1
            else:
                node_path.extend(node_pathi)
                k += ind + 1

        return node_path, interp_positions[:, 2]

    @classmethod
    def map_track_set_to_roads(cls,
                               track_set,
                               road_network,
                               stepsize=5,
                               obs_noise=5,
                               obs_noise_ne=5,
                               max_dist_init=1000,
                               max_dist=5,
                               min_prob_norm=0.5,
                               non_emitting_states=False,
                               non_emitting_length_factor=0.75,
                               max_lattice_width=10,
                               dist_noise=5,
                               dist_noise_ne=5,
                               restrained_ne=True,
                               avoid_goingback=True,
                               unique=False):
        """Map a set of tracks to a road network

        :return: A new RoadMapping object initialized with the full
        and simplified paths from the mapping.  These paths are
        dictionaries with track ID as key, and an array of road edges
        and corresponding timestamps
        """
        map_matching_graph = cls.build_map_matching_graph(road_network)

        # Path along road_network.simplified_network
        simplified_paths = {}
        # Path along road_network.network
        full_paths = {}
        for i, (track_id, track) in enumerate(sorted(track_set.items()), 1):
            print(f"Mapping track: {track_id} ({i}/{len(track_set)})")
            stepsize_temp = stepsize
            max_dist_temp = max_dist
            retried = False
            track_positions = np.array(
                [(*road_network._base_layer.meters_from_lon_lat(d.lon, d.lat),
                  d.timestamp) for d in track.detections])

            while True:
                try:
                    path, times = cls.map_track_to_roads(
                        track_positions,
                        map_matching_graph,
                        max_dist=max_dist_temp,
                        stepsize=stepsize_temp,
                        obs_noise=5,
                        obs_noise_ne=5,
                        max_dist_init=1000,
                        min_prob_norm=0.5,
                        non_emitting_states=False,
                        non_emitting_length_factor=0.75,
                        max_lattice_width=10,
                        dist_noise=5,
                        dist_noise_ne=5,
                        restrained_ne=True,
                        avoid_goingback=True)

                    # Simplify path
                    simplified_path = [
                        road_network.full_edge_to_simplified_edge(p)
                        if p is not None
                        else None
                        for p in path]

                    if unique:
                        full_paths[track_id] = cls._unique_paths(path, times)
                        simplified_paths[track_id] = cls._unique_paths(
                            simplified_path, times)
                    else:
                        full_paths[track_id] = [path, times]
                        simplified_paths[track_id] = [simplified_path, times]

                except Exception as e:
                    if not retried or stepsize_temp > 0.5:
                        max_dist_temp *= 2
                        stepsize_temp /= 2
                        retried = True
                        print('Trying step size', stepsize_temp,
                              'and max_dist', max_dist_temp)
                        continue

                    print(e)

                break

        mapping = cls()
        mapping.full_paths = full_paths
        mapping.simplified_paths = simplified_paths

        return mapping

    def visualize_full_paths(self,
                             outdir,
                             road_network,
                             track_set,
                             track_ids=None,
                             zoom_to_track=False):
        """ Plots full paths that have been mapped onto the road network.

        :param track_ids: Set of track IDs to plot (default=None)

        :param zoom_to_track: If True, zoom the plot to the road
        mapping for the given track

        Generates one plot file per track.
        """
        if not os.path.isdir(outdir):
            pathlib.Path(outdir).mkdir(parents=True, exist_ok=True)

        aspect_ratio = (road_network._base_layer.res_x /
                        road_network._base_layer.res_y)

        G = road_network.network
        node_pos = dict(zip(G.node.keys(),
                            [(G.node[x]['easting'], G.node[x]['northing'])
                             for x in G.node.keys()]))
        full_paths = self.full_paths
        if track_ids is not None:
            full_paths = {track_id: v for track_id, v in full_paths.items()
                          if track_id in track_ids}
        for i, item in enumerate(sorted(full_paths.items()), 1):
            track_id, (path, time) = item
            print(f"Plotting track: {track_id} ({i}/{len(full_paths)})")

            plt.clf()
            plt.cla()
            fgr = plt.figure(dpi=240)

            nx.draw_networkx_edges(G, node_pos, arrows=False)
            nodelist = [n[:2] for n in path if n is not None]
            nodelist = list(set(chain.from_iterable(nodelist)))

            # Plot raw track positions
            min_x, max_x, min_y, max_y = None, None, None, None
            for d in track_set[track_id].detections:
                x, y = road_network._base_layer.meters_from_lon_lat(
                    d.lon, d.lat)
                min_x = x if min_x is None or x < min_x else min_x
                min_y = y if min_y is None or y < min_y else min_y
                max_x = x if max_x is None or x > max_x else max_x
                max_y = y if max_y is None or y > max_y else max_y
                plt.plot(x, y, 'b-', linewidth=1, zorder=1)
                plt.plot(x, y, 'rx', markersize=1, zorder=1)

            if zoom_to_track:
                w = max_x - min_x
                h = max_y - min_y
                x_pad = max(((aspect_ratio * h * 1.1) - w) / 2, w * 0.1)
                y_pad = max((((w * 1.1) / aspect_ratio) - h) / 2, h * 0.1)

                plt.xlim(min_x - x_pad, max_x + x_pad)
                plt.ylim(min_y - y_pad, max_y + y_pad)

            nx.draw_networkx_nodes(G,
                                   node_pos,
                                   nodelist=nodelist,
                                   node_color='b',
                                   node_size=1)

            plt.xlabel('Easting (meters)', fontsize=12)
            plt.ylabel('Northing (meters)', fontsize=12)
            plt.savefig(os.path.join(outdir, f"road_map_path_{track_id}"))
            plt.close(fgr)

    def save_csv(self, path):
        """Save the given full and simplified paths properties of this
        RoadMapping instance (as computed by
        RoadMapping.map_track_set_to_roads) to the given path in a
        CSV-like format (see load_road_mapping for documentation of
        the format).
        """
        with open(path, 'w', newline='') as f:
            csv_f = csv.writer(f)
            for type_, paths in [('full', self.full_paths),
                                 ('simp', self.simplified_paths)]:
                for track_id, (edges, times) in paths.items():
                    for edge, time in zip(edges, times):
                        if not (edge is None or len(edge) in (2, 3)):
                            raise NotImplementedError(
                                "Unexpected edge: " + repr(edge))
                        csv_f.writerow([type_,
                                        track_id,
                                        time,
                                        *(edge or ['null'])])

    @classmethod
    def from_csv(cls, path):
        """Read the given CSV-like file to create a RoadMapping
        object, and set the full and simplified path dictionaries, as
        computed by RoadMapping.map_track_set_to_roads.  The CSV
        should have four through six columns, where the first has
        "full" or "simp" for a full or simplified path entry, the
        second has the track UID, the third has the time and the
        fourth through last has a path edge, or only four columns
        exactly when the fourth column is "null".  Returns a new
        RoadMapping object containing the full paths dictionary and
        the simplified paths dictionary.
        """
        with open(path, newline='') as f:
            csv_f = csv.reader(f)
            mapping = cls()
            for row in csv_f:
                if not (4 <= len(row) <= 6):
                    raise ValueError("Expected 4 to 6 values in row but "
                                     "received {}: {}".format(len(row), row))
                type_, track_id, time, *edge = row
                if len(edge) == 1:
                    if edge[0] != 'null':
                        raise ValueError("Row has one-long path entry that "
                                         "isn't 'null': {}".format(row))
                    edge = ()

                if type_ == "full":
                    edges, times = mapping.full_paths.setdefault(
                        int(track_id), [[], []])
                elif type_ == "simp":
                    edges, times = mapping.simplified_paths.setdefault(
                        int(track_id), [[], []])
                else:
                    raise ValueError("Invalid type {!r}, expected 'full' or "
                                     "'simp'".format(type_))
                edges.append(tuple(map(int, edge)) or None)
                times.append(float(time))

        return mapping


if __name__ == "__main__":
    road_network = RoadNetwork.from_osm("./data/net_quantico/osm_bbox.osm.xml")

    kw18_in = kw_utils.read_kw18("./data/net_quantico/routes/traffic0/high_density/run0/sumo.kw18")

    # For testing, use track '0' and track '1'
    subset_tracks = {k: v for k, v in kw18_in.items() if k in set([0, 1])}

    mapping = RoadMapping.map_track_set_to_roads(subset_tracks,
                                                 road_network)

    mapping.visualize_full_paths("tmp",
                                 road_network,
                                 subset_tracks,
                                 zoom_to_track=False)
