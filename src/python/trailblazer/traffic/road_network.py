# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import string
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from scipy.interpolate import interp1d
from scipy.spatial import cKDTree
import networkx as nx
import xml.etree.ElementTree as xml
from math import sqrt

from trailblazer.utils import coordinate_converter as cc
from trailblazer.traffic.base_layer import BaseLayer

class RoadNetwork( object ):
    """Encode a road network as a graph.

    """
    def __init__(self):
        self._network = None
        self._lon0 = None
        self._lat0 = None
        self._kdtree = None
        self._network = None
        self._simplified_network = None
        self._base_layer = None

    @property
    def network(self):
        """Return NetworkX DiGraph model of the full road network.

        The road network is defined by a graph with nodes connected by edges.
        Nodes can represent sinks, sources, or intersections. However, there
        may also be sections of roadways where a sequence of nodes is defined
        without oppurtunity for branching (i.e., path graph) simply to model a
        non-linear section of road with piecewise-linear sequence of edges.
        A vehicle entering such a sequence has the choice of proceeding forward
        or backwards but not cannot leave this path subgraph except at the
        entrance of exit nodes (assuming the vehicle doesn't go off road). A
        simplified version of this network with these path sugraphs removed is
        provided by 'simplified_network,' in which each nodes is a source,
        sink, and/or intersection.

        :return: Simple directed network encoding all roads with the road
            network. Piecewise-linear roads are defined by an array of nodes
            connected with edges. Each node stores the latitude and longitude
            (degrees) for its position in the world, and an edge connecting two
            nodes defines a linear section of road that a car can drive on.
            Edges (node1,node2) are directed, indicating that a car is allowed
            to move from node1 to node2. Therefore, the definition for a two-
            way road would be encoded with an additional (node2,node1) edge.
            Curved sections of road may require a long sequence of linear edges
            to approximate the curve.
        :rtype: networkx.DiGraph

        """
        return self._network

    @property
    def simplified_network(self):
        """Return NetworkX DiGraph model of the simplified road network.

        This is a simplified version of the full road network, 'network', with
        path sugraphs simplified to a single edge.

        :return: Simplified version of 'network' with each path graph subgraph
            (https://en.wikipedia.org/wiki/Path_graph) removed and replaced
            with an edge connecting its endpoints. A path graph is a sequence
            of nodes where each node in the path, except for the endpoints, has
            two neighbors. Therefore, a vehicle traversing this path subgraph
            has a deterministic trajectory, unless a u-turn is made. In other
            words, this simplified graph has edges that connect only sources,
            sinks, or intersections. This graph is useful in understanding
            behaviors of vehicles because changing from one edge to another
            required a decision to be made at an intersection. This network can
            be linked back to 'network' with the dictionary 'path_subgraphs',
            which accepts and edge in 'simplified_network' and returns the node
            path in 'network' that it replaces.
        :rtype: networkx.DiGraph

        """
        return self._simplified_network

    @property
    def path_subgraphs(self):
        """Return path graph node dictionary reduced in 'simplified_network'.

        'path_subgraphs' accepts keys that are 'simplified_network' edges that
        are not in 'network' (i.e., they are a simplification of a path) and
        returns the sequence of 'network' nodes that was simplied.

        """
        return self._path_subgraphs

    def full_edge_to_simplified_edge(self, edge):
        """Return simplified network edge from full-network edge.

        If the edge is not one that has been simplified in the simplified
        network, then the input edge is returned.

        """
        try:
            return self._inverse_path_subgraph_map[edge]
        except KeyError:
            # This edge was not simplified.
            return edge

    @property
    def lat0(self):
        """Latitude (degrees) at origin of easting/northing coordinate system.

        """
        return self._lat0

    @property
    def lon0(self):
        """Longitude (degrees) at origin of easting/northing coordinate system.

        """
        return self._lon0

    @lat0.setter
    def lat0(self, val):
        self._lat0 = val

    @lon0.setter
    def lon0(self, val):
        self._lon0 = val

    @staticmethod
    def from_osm(path_to_file):
        """Parse an osm file and return RoadNetwork object.

        Modified from initial work by Patrick Klose (December, 10th 2017).
        http://www.patrickklose.com/posts/parsing-osm-data-with-python/

        Parameters:
            filename - The filename of the file to import.
        Returns:
            graph - The created graph.

        """
        # Parse the xml structure and initialize variables.
        e = xml.parse(path_to_file).getroot()
        node_dict_tmp = {}
        G = nx.DiGraph()

        # Allow these types of streets to be represented in the network by an edge.
        way_types = ['motorway', 'track', 'trunk', 'trunk_link', 'primary', 'secondary', 'tertiary',
                     'unclassified', 'residential', 'service', 'living_street', 'road',
                     'motorway_link', 'primary_link', 'secondary_link', 'tertiary_link', 'corridor']

        unused_way_types = set()
        # Create nodes and edges.
        for i in e:
            # Nodes.
            if i.tag == "node":
                node_dict_tmp[int(i.attrib["id"])] = [i.attrib["lat"],i.attrib["lon"]]

            # Edges.
            if i.tag == "way":
                # Define all attributes of the way.
                attr_dict = {}

                # Set defaults
                directed = False
                attr_dict['max_speed'] = None
                attr_dict['way_type'] = None

                way_tmp = []
                for j in i:
                    if j.tag == "nd":
                        way_tmp.append(int(j.attrib["ref"]))
                    if j.tag == "tag":
                        if j.attrib["k"] == "oneway":
                            directed = j.attrib["v"] == "yes"
                        elif j.attrib["k"] == "highway":
                            attr_dict['way_type'] = j.attrib["v"]
                        elif j.attrib["k"] == "name":
                            attr_dict['name'] = j.attrib["v"]
                        elif j.attrib["k"] == "lanes":
                            try:
                                attr_dict['lanes'] = int(j.attrib["v"])
                            except ValueError:
                                pass

                        elif j.attrib["k"] == "maxspeed":
                            v = j.attrib["v"]
                            if 'mph' in v:
                                # Convert to m/s.
                                v = float(v.split(' mph')[0])*0.44704
                            elif 'kph' in v:
                                # Convert to m/s.
                                v = float(v.split(' mph')[0])*0.277778
                            else:
                                raise Exception('Unhandled maxspeed value: ' % v)

                            if v >= 0:
                                attr_dict['max_speed'] = v

                if attr_dict['way_type'] in way_types:
                    G.add_path(way_tmp, **attr_dict)
                    if not directed:
                        G.add_path(list(reversed(way_tmp)), **attr_dict)
                else:
                    unused_way_types.add(attr_dict['way_type'])

        unused_way_types = list(unused_way_types)
        if len(unused_way_types) > 0:
            print('These way types were not included:', unused_way_types)


        # Extend the nodes by their geographical coordinates.
        network_nodes = G.nodes()
        for i in network_nodes:
            current_node_coords = node_dict_tmp[i]
            G.node[i]['latitude'] = float(current_node_coords[0])
            G.node[i]['longitude'] = float(current_node_coords[1])

        road_network = RoadNetwork.from_networkx(G)

        base_layer = BaseLayer.from_road_network(road_network)
        road_network._base_layer = base_layer
        road_network.lat0 = base_layer.lat0
        road_network.lon0 = base_layer.lon0
        road_network.populate_meters_from_lon_lat()

        return road_network

    @staticmethod
    def from_networkx(G):
        """Return RoadNetwork from a networkx graph.

        """
        # Remove self loops.
        null_nodes = [n for n in G.nodes_with_selfloops()]
        G.remove_nodes_from(null_nodes)

        # Remove nodes that do not belong to an edge.
        G.remove_nodes_from(list(nx.isolates(G)))

        G0 = G.copy()
        G0_undirected = G0.to_undirected()

        # A simplified network can have multiple paths between intersection
        # nodes. So, we have to use a MultiGraph.
        G = nx.MultiGraph()
        G.add_nodes_from(G0_undirected.nodes(data=True))
        G.add_edges_from(G0_undirected.edges);

        # Create a simplied version of the road network graph connecting
        # all sources/sinks and intersections.
        source_sink_nodes = []
        intersection_nodes = []
        normal_nodes = []
        node_classification = {}
        for n in G.nodes():
            num_neighbors = len(list(G.neighbors(n)))
            if num_neighbors == 1:
                source_sink_nodes.append(n)
                node_classification[n] = 'source/sink'
            elif num_neighbors == 2:
                normal_nodes.append(n)
                node_classification[n] = 'normal'
            else:
                intersection_nodes.append(n)
                node_classification[n] = 'intersection'

        # Produce a simplied network removing all subgraphs that are path
        # graphs (https://en.wikipedia.org/wiki/Path_graph) and simply
        # connecting the endpoints of the path. A path graph is a sequence of
        # nodes where each node in the path, except for the endpoints, has two
        # neighbors. Therefore, a vehicle traversing this path subgraph has a
        # deterministic trajectory. In other words, we want to remove sections
        # of the graph that are simply one- or two-way roads leading between
        # sources, sinks, or intersections. After processing starting from
        # source/sink and then intersections, there should be no normal nodes
        # left in the graph. However, if there are isolated closed loops, these
        # will not have been removed. So, we do a final pass over each of the
        # normal nodes.
        # Note: this is also known as chain decomposition.
        endpoint_class = ['source/sink','intersection']
        removed_nodes = []
        path_subgraphs = {}
        for n in source_sink_nodes + intersection_nodes + normal_nodes:
            # n is the starting-point node.
            if not G.has_node(n):
                # This applies for when we do the final pass on the
                # 'normal_nodes'.
                continue

            for ni in list(G.neighbors(n)):
                # Our stating-point node, n, may be an intersection. So, we
                # will separately reduce along each direction that can be taken
                # at the intersection.
                if not G.has_node(ni):
                    # It is possible in the case of loops that the node was
                    # already deleted during a previous reduction.
                    continue

                path_subgraph_nodes = [n,ni]
                first_loop = True
                while ni != n and node_classification[ni] not in endpoint_class:
                    # Follow the source until it hits a sink/source or
                    # intersection.
                    neighbors = list(G.neighbors(ni))

                    if first_loop:
                        # We are considering a neighbor of a neighbor of n, so
                        # we don't want to end up going back to n on this first
                        # loop.
                        neighbors = set(neighbors) - set([n])
                        first_loop = False

                    if len(neighbors) != 1:
                        # If a loop was reduced and the ni becomes n, then
                        # there will be zero neighbors. This can also occur if
                        # a sink/source is reached. If neighbors are greater
                        # than 2, then an intersection was reached.
                        break

                    removed_nodes.append(ni)
                    G.remove_node(ni)
                    ni = list(neighbors)[0]
                    path_subgraph_nodes.append(ni)

                if first_loop:
                    # There were no removed nodes, so no paths were simplified.
                    continue

                # ni is now a sink, source, or intersection. We will connect it
                # back to n, which is where we started.
                k = G.add_edge(n, ni)

                # Store off the path subgraph node sequence that was stripped
                # from G and replaced with edge (n,ni). We number the edges
                # because it is possible there are multiple simplified ways to
                # get between nodes n and ni.
                path_subgraphs[(n,ni,k)] = path_subgraph_nodes

                if n == ni:
                    # This is the case of a loop, which may be traversed in
                    # either direction. Therefore, we need to make sure we can
                    # distinguish between the two directions even though
                    # n -> ni is equivalent to ni -> n. So, we need to
                    # increment k.
                    k = G.add_edge(n, ni)

                path_subgraphs[(ni,n,k)] = list(reversed(path_subgraph_nodes))

        """
        plt.close('all')
        self.plot()
        plt.scatter(pos[n][0], pos[n][1], s=400, c='g')
        plt.scatter(pos[ni][0], pos[ni][1], s=400, c='r')
        nx.draw_networkx_labels(self._network, pos)
        nx.draw_networkx_nodes(G, pos, nodelist=removed_nodes,
                               node_color='k', node_size=50)
        """

        # We turned the road network into an un_directed one (i.e., all two-way
        # streets) for simplicity in the above steps. However, we want to
        # remove edges that are going the wrong way down a one-way street.
        #edges = G.edges(keys=True)
        G = G.to_directed()
        for edge in list(G.edges(keys=True)):
            try:
                path = path_subgraphs[edge]
            except KeyError:
                # Was an original edge in G0, not one that was simplified.
                if not G0.has_edge(edge[0], edge[1]):
                    # Was one-way road in G0 going the other direction.
                    G.remove_edge(*edge)

                continue

            # Check that this path exists in G0 and does not represent a
            # traversal down the wrong direction of a one-way road. There is a
            # corner case that can occur when a path between a pair of sinks/
            # sources/intersections changes from a two-way to one-way road
            # along the path in G0. All of the sub-edges on the path were
            # removed from G, so if we remove the edge in G representing the
            # path in G0, then we will end up with edges in G0 that have no
            # correspondence to an edge in G or in path_subgraphs. Here, we
            # will handle this case by assuming that if any segment of the
            # simplified path allows movement in a particular direction, then
            # the whole simplified edge can allow that direction of motion.
            remove_path = True
            for n in range(len(path)-1):
                if G0.has_edge(path[n], path[n+1]):
                    remove_path = False
                    break

            if remove_path:
                G.remove_edge(edge[0], edge[1])
                del path_subgraphs[edge]

        # Return the generated graph.
        network = RoadNetwork()
        network._network = G0
        network._simplified_network = G
        network._path_subgraphs = path_subgraphs
        network._source_sink_nodes = source_sink_nodes
        network._intersection_nodes = intersection_nodes

        network._inverse_path_subgraph_map = {}
        for key in path_subgraphs:
            path = path_subgraphs[key]
            for i in range(len(path) - 1):
                edge = tuple(path[i:i+2])
                network._inverse_path_subgraph_map[edge] = key

        # --------------------------------------------------------------------
        # Populate the length of each edge (meters).
        edges0 = G0.edges()
        temp_length = {}
        for edge in edges0:
            disp = cc.llh_to_enu(G0.node[edge[0]]['latitude'],
                              G0.node[edge[0]]['longitude'], 0,
                              G0.node[edge[1]]['latitude'],
                              G0.node[edge[1]]['longitude'], 0,
                              in_degrees=True)[:2]
            d = np.linalg.norm(disp)
            edges0[edge]['length'] = d
            temp_length[edge] = temp_length[(edge[1],edge[0])] = d

        edges = G.edges(keys=True)
        for edge in edges:
            try:
                path = path_subgraphs[edge]
                d = [temp_length[(path[i], path[i+1])]
                     for i in range(len(path)-1)]
                edges[edge]['length'] = sum(d)
            except KeyError:
                # Was an original edge in G0, not one that was simplified.
                edges[edge]['length'] = edges0[(edge[0],edge[1])]['length']
        # --------------------------------------------------------------------

        return network

    def node_position(self, nodes=None, xaxis='longitude', yaxis='latitude'):
        """Plot the road network.

        """
        G = self._network
        if nodes is None:
            nodes = G.node.keys()

        pos = [(G.node[x][xaxis],G.node[x][yaxis]) for x in nodes]
        return pos

    def plot_edge(self, edge, xaxis='longitude', yaxis='latitude',
                  edge_color='k', draw_nodes=True):
        """Plot a single edge of the road network.

        """
        G = self._network
        pos = dict(zip(G.node.keys(),
                       [(G.node[x][xaxis],G.node[x][yaxis])
                       for x in G.node.keys()]))
        edges = G.edges()
        lanes = [int(G.get_edge_data(*edge).get('lanes', 1)) for edge in edges]
        nx.draw_networkx_edges(G, pos, width=lanes, edge_color=edge_color,
                               arrows=False)

    def plot(self, edgelist=None, xaxis='longitude', yaxis='latitude',
             edge_color='k', width=None, draw_nodes=True, show_legend=True,
             label_nodes=False):
        """Plot the road network.

        """
        G = self._network
        pos = dict(zip(G.node.keys(),
                       [(G.node[x][xaxis],G.node[x][yaxis])
                       for x in G.node.keys()]))
        edges = G.edges()
        if width is None:
            # Set to number of lanes.
            width = [int(G.get_edge_data(*edge).get('lanes', 1))
            for edge in edges]

        nx.draw_networkx_edges(G, pos, edgelist=edgelist, width=width,
                               edge_color=edge_color, arrows=False)

        if draw_nodes:
            if edgelist is not None:
                nodelist = list(zip(*edgelist))
                nodelist = list(set(nodelist[0]) | set(nodelist[1]))
            else:
                nodelist = G.nodes()

            if show_legend:
                in_degree = G.in_degree()
                out_degree = G.out_degree()
                node_color = []
                for n in nodelist:
                    num_neighbors = len(list(G.neighbors(n)))
                    if in_degree[n] == 0:
                        # Source.
                        node_color.append((1,0,1))
                    elif out_degree[n] == 0:
                        # Sink.
                        node_color.append((0,1,0))
                    elif num_neighbors == 1:
                        if in_degree[n] == 1 and out_degree[n] == 1 and \
                           n in G.neighbors(list(G.neighbors(n))[0]):
                               # Source/Sink
                               node_color.append((0,0,1))
                        else:
                            # One-Way Road
                            node_color.append((1,1,0))
                    elif num_neighbors == 2:
                        # Two-Way Road.
                        node_color.append((1,0,0))
                    else:
                        # Intersection.
                        node_color.append((0,1,1))

                # Explicitly define the legend.
                line1 = mlines.Line2D([], [], color=(1,0,1), marker='o',
                                      markersize=15, label='Source')
                line2 = mlines.Line2D([], [], color=(0,1,0), marker='o',
                                      markersize=15, label='Sink`')
                line3 = mlines.Line2D([], [], color=(0,0,1), marker='o',
                                      markersize=15, label='Source/Sink`')
                line4 = mlines.Line2D([], [], color=(1,1,0), marker='o',
                                      markersize=15, label='One-Way Road')
                line5 = mlines.Line2D([], [], color=(1,0,0), marker='o',
                                      markersize=15, label='Two-Way Road')
                line6 = mlines.Line2D([], [], color=(0,1,1), marker='o',
                                      markersize=15, label='Intersection')

                plt.legend(handles=[line1,line2,line3,line4,line5,line6])
            else:
                node_color = [(0,0,0) for n in nodelist]

            nx.draw_networkx_nodes(G, pos, nodelist=nodelist,
                                   node_color=node_color, node_size=10)

            if label_nodes:
                labels = {t:str(t) for t in nodelist}
                nx.draw_networkx_labels(G, pos, labels=labels)

    def populate_meters_from_lon_lat(self):
        """Populate easting and northing (meters) from longitude and latitude.

        This method will populate key/value pairs for 'easting' and 'northing'
        for the node definitions.

        """
        if self.lat0 is None or self.lon0 is None:
            raise Exception('Must populate attributes \'lat0\' and \'lon0\' '
                            'first.')

        G = self.network
        for node in G.nodes():
            lon,lat = G.node[node]['longitude'],G.node[node]['latitude']
            enu = cc.llh_to_enu(lat, lon, 0, self.lat0, self.lon0, 0,
                             in_degrees=True)
            G.node[node]['easting'] = enu[0]
            G.node[node]['northing'] = enu[1]

    def crop(self, lon_range, lat_range):
        """Crop out road segments completely outside of the provided domain.

        For each road segment that starts outside of the domain defined by the

        :param lon_range: The minimum and maximum and longitudes (degrees) in
            the domain to crop to.
        :type lon_range: 2-array of float

        :param lat_range: The minimum and maximum and latitudes (in degrees) in
            the domain to crop to.
        :type lat_range: 2-array of float

        """
        G = self.network.copy()

        lon_range = [min(lon_range),max(lon_range)]
        lat_range = [min(lat_range),max(lat_range)]

        def node_lon_lat(n):
            return G.node[n]['longitude'],G.node[n]['latitude']

        def outside_domain(n):
            """Return whether a node is outside the domain.

            """
            lon,lat = node_lon_lat(n)

            if lon < lon_range[0] or lon > lon_range[1] or \
               lat < lat_range[0] or lat > lat_range[1]:
                return True

            return False

        outside_nodes = set()
        for n in G.nodes():
            if outside_domain(n):
                outside_nodes.add(n)

        for n in outside_nodes:
            neighbors = list(G.neighbors(n))
            for n2 in neighbors:
                # Check to see whether this edge is completely outside of the
                # domain.
                lon0,lat0 = node_lon_lat(n)
                lon1,lat1 = node_lon_lat(n2)

                b = big.utilities.line_segment_intersect_rectangle(lon0, lat0,
                                                                   lon1, lat1,
                                                                   lon_range[0],
                                                                   lat_range[0],
                                                                   lon_range[1],
                                                                   lat_range[1])

                if False:
                    plt.plot([lon0,lon1], [lat0,lat1], '-o')
                    plt.plot([lon_range[0],lon_range[1],lon_range[1],
                              lon_range[0],lon_range[0]],
                             [lat_range[0],lat_range[0],lat_range[1],
                              lat_range[1],lat_range[0]], '-o')

                if not b:
                    G.remove_edge(n, n2)

        return RoadNetwork.from_networkx(G)


    def lat_lon_bounds(self):
        """Return latitude and longitude (degrees) bounds of the road network.

        """
        G = self._network
        latitudes = [node['latitude'] for node in G.node.values()]
        longitudes = [node['longitude'] for node in G.node.values()]

        min_lat, max_lat = min(latitudes), max(latitudes)
        min_lon, max_lon = min(longitudes), max(longitudes)

        return [min_lat,max_lat],[min_lon,max_lon]


def angle_difference(a1, a2):
    """Angular distance between two angles (radians).

    """
    return min(abs(a1-a2), abs(a1-a2) % (2*np.pi), abs(a2-a1) % (2*np.pi),
               abs(a2-a1))


class RoadSegmentNotUsed(object):
    """An extent of road without any oppurtunities to turn off the road.

    """
    def __init__(self, x, y, discretization=1, two_way=False):
        """Create road segment.

        :param x: Easting coordinates (meters).
        :param y: array

        :param y: Northing coordinates (meters).
        :type y: array

        :param discretization: The road is discretized with this maximum
            distance between segments to facilitate nearest-node calculations.
        :type discretization: float

        :param two_way:
        :type: bool

        """
        self._x0 = x
        self._y0 = y
        ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        arclen = np.cumsum(ds)
        arclen = np.hstack([0, arclen])

        arclen[-1]
        s = np.linspace(0, arclen[-1],
                        int(np.ceil(arclen[-1]/discretization)) + 1)

        fx = interp1d(arclen, x, 'slinear', copy=False,
                      fill_value='extrapolate', assume_sorted=True)

        fy = interp1d(arclen, y, 'slinear', copy=False,
                      fill_value='extrapolate', assume_sorted=True)

        self._s = s
        self._x = fx(s)
        self._y = fy(s)
        self._fx = fx
        self._fy = fy
        self._discretization = discretization
        self._direction = None
        self._kdtree = None
        self._heading = None

    @property
    def s(self):
        """Return arclength corresponding to each x, y.

        """
        return self._s

    @property
    def x(self):
        """Return x coordinates (meters).

        """
        return self._x

    @property
    def y(self):
        """Return y coordinates (meters).

        """
        return self._y

    @property
    def length_meters(self):
        return self._s[-1]

    def xy_at_path_length(self, s):
        """Return the (x, y) coordinates at a traveled distance s.

        """
        return self._fx(s), self._fy(s)

    @property
    def direction(self):
        """Spline function accepting arclength and returning local unit vector.

        """
        if self._direction is None:
            dx = np.diff(self.x)
            dy = np.diff(self.y)
            dxy = np.vstack([dx,dy])
            dxy = dxy/np.sqrt(np.sum(dxy**2, 0))
            direction = np.zeros((2,len(self.x)))
            direction[:,:-1] = dxy
            direction[:,1:] += dxy
            self._direction = direction/np.sqrt(np.sum(direction**2, 0))

        return self._direction

    @property
    def heading(self):
        """Heading (degrees) associated with each node.

        """
        if self._heading is None:
            self._heading = np.arctan(self.direction[0], self.direction[1])

        return self._heading

    @property
    def kdtree(self):
        if self._kdtree is None:
            self._kdxy = np.vstack([self.x,self.y]).T
            self._kdtree = cKDTree(self._kdxy, leafsize=20, compact_nodes=True,
                                   copy_data=False, balanced_tree=True)

        return self._kdtree

    def closest_point(self, xy, r=float('inf')):
        """Return details about closest point on road to the provided points.

        :param xy: Point to consider the distance to closest point on the road
            segment.
        :type xy: array-like shape: (2,num_points)

        :param r: Radius within each point in 'xy' to consider matches
            (meters).
        :type r: float

        :return dist: Index into self.x, self.y, self.s, self.direction for the
            closest point. The second element is the distance of closest approach.
        :rtype: array-like shape: (num_points)

        """
        dist,i = self.kdtree.query(xy, distance_upper_bound=r)
        return i,dist

    def nodes_within_distance(self, xy, r):
        """Return details about closest point on road to the provided points.

        :param xy: Point to consider the distance to closest point on the road
            segment.
        :type xy: array-like shape: (2,num_points)

        :param r: Radius within each point in 'xy' to consider matches
            (meters).
        :type r: float

        :return: Index into self.x, self.y, self.s, self.direction for the
            closest point. The second element is the distance of closest approach.
        :rtype: array-like shape: (num_points)

        """
        ind = self.kdtree.query_ball_point(xy, r=r)
        return ind

    def match_trajectory(self, xy, heading, speed, r):
        """Return details about track match with road segment.

        :param xy: Array of trajectory positions (meters).
        :type xy: array-like shape: (2,num_points)

        :param v: Heading measured clockwise from north (degrees).
        :type v: array-like shape: (2,num_points)

        :param speed: Speed (meters/s).
        :type speed: array-like shape: (2,num_points)

        :param r: Radius within each point in 'xy' to consider matches
            (meters).
        :type r: float

        :return: Most-likely arclength along the road corresponding that the
            track is located and the associated confidence for each assessment.
            If no viable results were found, None is returned.
        :rtype: [arclengths,confidences]

        """
        inds = self.nodes_within_distance(xy, r)

        if not np.any([len(ind) > 0 for ind in inds]):
            return None

        ret = []

        for i in range(len(inds)):
            if len(inds[i]) == 0:
                ret.append([])
                continue

            # Distance (m) between road nodes and trajectory state i.
            d = [sqrt((self.x[k]-xy[i,0])**2 + (self.y[k]-xy[i,1])**2)
                 for k in inds[i]]
            d = np.array(d)

            # Component of confidence due to distance from the road.
            dconf = np.exp( -(d / r)**2*2)

            if self.two_way:
                dh = [min(angle_difference(self.heading[k], heading[i]),
                          angle_difference(self.heading[k], -heading[i]))
                      for k in inds[i]]
            else:
                dh = [angle_difference(self.heading[k], heading[i])
                      for k in inds[i]]

            dh = np.array(dh)

            # Component of confidence due to angle from the road.
            dhconf = np.exp( -(dh / np.pi)**2*2)

            conf = dconf*dhconf

            ind = np.argmin(conf)
            s = self.s[inds[i]]
            conf = conf[ind]

            ret.append([s,conf])

        return ret

    def track_divergence_locs(self, track, dist_thresh1=10, dist_thresh2=20,
                              dist_thresh3=10, angle_thresh=90,
                              plot_results=False):
        """
        :param angle_thresh: Maximum direction alignment error (degrees)
            between the road direction and the track.
        :type angle_thresh: float

        """
        xy = np.vstack([self.x,self.y]).T
        dist,xy2,track_dir = track.closest_point(xy)

        ind = dist > dist_thresh1


        dp = np.sum(self.direction*track_dir.T, 0)
        dp = np.maximum(np.minimum(dp, 1), -1)
        match_angle = np.arccos(dp)*180/np.pi

        dist,match_angle = self.match_track(track)

        # Positive ind is valid, negative ind is invalid.
        ind = np.logical_and(match_angle < angle_thresh, dist < dist_thresh1)
        ind = ind.astype(np.int)
        ind = np.argwhere(np.diff(ind) != 0)[:,0]
        xy = np.vstack([self.x[ind],self.y[ind]])

        if plot_results:
            plt.figure()
            #s = np.linspace(0, len(dist)*2, len(dist)+1)[:-1]
            #plt.plot(s[ind], dist[ind], 'go')
            #plt.plot(s[~ind], dist[~ind], 'ro')

            track.plot_trajectory(coordinates='meters')
            plt.plot(self.x, self.y)
            plt.scatter(xy[0], xy[1], c='r', s=100)

        return xy.T

    def plot(self):
        plt.plot(self._x0, self._y0)


def turns_from_paths(paths, tmin=None, tmax=None):
    """Return turns associated with paths.

    We start with a set of Track objects, and we want to map these tracks to
    turns at intersections of a RoadNetwork object. For each Track object, we
    can map its trajectory to the road network segments using the RoadNetwork
    method 'map_track_to_roads' with unique=True to get the 'path' and 'times'
    output. We can store [path,times] in a dictionary with the key being the
    associated track UID. This funciton accepts that 'paths' dictionary and
    returns information about what turns were made and their frequency.

    :param paths: Dictionary with keys being the track uid string and values
        being the output of RoadNetwork's map_track_to_roads method with
        'unique' set to true.
    :type paths: dictionary

    :param tmin: Minimum time (s) to consider for a turn.
    :type tmin: float

    :param tmax: Maximum time (s) to consider for a turn.
    :type tmax: float

    :return: List with three elements. The first element (turns_by_track) is a
        dictionary with keys being the track UID and values are a dictionary
        with keys being times and values being the turn that was made. The
        second element (turns) is a dictionary accepting a pair of
        RoadNetwork.simplified_network edges representing a transition at an
        intersection and returning a list of track UID and associated time that
        the track made the specified turn. The third element is a networkx
        DiGraph representing all of the observed turns with each node being a
        RoadNetwork.simplified_network edge (i.e., road segment) and each edge
        being a pair of RoadNetwork.simplified_network edges (i.e., pair of
        road segments)
    :rtype:  list of dictionary and networkx DiGraph

    """
    # Define a network where nodes are road segments and edges are transitions
    # between the segments (i.e., valid turns).
    for track_uid in paths:
        path = paths[track_uid]

    tracks_by_turn = {}
    turns_by_track = {}
    for track_uid in paths:
        path = paths[track_uid]
        turns_by_track[track_uid] = {}
        if len(path[0]) < 2:
            continue

        edges = path[0]
        times = path[1]
        for i in range(len(edges) - 1):
            if tmin is not None and times[i+1] < tmin:
                continue

            if tmax is not None and times[i+1] > tmax:
                continue

            turn_key = tuple(edges[i:i+2])
            if turn_key not in tracks_by_turn:
                tracks_by_turn[turn_key] = []

            turns_by_track[track_uid][times[i+1]] = turn_key
            tracks_by_turn[turn_key].append((track_uid,times[i+1]))

    return turns_by_track, tracks_by_turn


def create_turn_graph(paths):
    """Create graph encoding the frequency of turns between particular roads.

    :param paths: List of lists with each sub-list being a chronological
        sequence of road network edges encoding a track's trajectory. These
        trajectories should be used to probabilities of making particular turns
        given that a track start on one side of an intersection.
    :type paths: list of lists

    """
    turn_graph = nx.DiGraph()
    for edges in paths:
        for edge in edges:
            turn_graph.add_node(edge)

    # Give nodes reasonable names.
    l = int(np.ceil(np.log10(len(turn_graph.nodes))))
    names = [''.join([string.ascii_uppercase[int(s)] for s in str(i).zfill(l)])
             for i in range(len(turn_graph.nodes))]
    nx.set_node_attributes(turn_graph, names, 'name')

    num_veh_by_turn = {}
    for subpath in paths:
        for i in range(len(subpath) - 1):
            turn = (subpath[i], subpath[i + 1])
            if turn not in num_veh_by_turn:
                num_veh_by_turn[turn] = 0

            num_veh_by_turn[turn] += 1

    for turn in num_veh_by_turn:
        turn_graph.add_edge(turn[0], turn[1],
                            num_vehicles=num_veh_by_turn[turn])

    # Normalize number of tracks_by_turn into a probability.
    for node in turn_graph.nodes:
        neighbors = list(turn_graph.neighbors(node))
        w = np.sum([turn_graph[node][node2]['num_vehicles']
                    for node2 in neighbors])
        for node2 in neighbors:
            edge_data = turn_graph[node][node2]
            edge_data['probability'] = edge_data['num_vehicles']/w

    return turn_graph
