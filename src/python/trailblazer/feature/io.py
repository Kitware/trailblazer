import csv
from itertools import repeat
import os

import networkx.drawing.nx_pylab as nx_draw
import numpy
import tqdm

from trailblazer.traffic.base_layer import BaseLayer
from trailblazer.utils.kw_utils import read_kw18
from trailblazer.traffic.road_mapping import RoadMapping
from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.traffic.vital import TrackSet

from trailblazer.feature import features

def feature_input_from_files(
        track_set, road_mapping, road_network, base_layer,
        time_step, window_length, time_bounds=None,
):
    """Create a FeatureInput using files to provide each object (except
    the time step, window length, and time bounds)

    """
    return features.FeatureInput(
        read_kw18(track_set),
        RoadMapping.from_csv(road_mapping),
        RoadNetwork.from_osm(road_network),
        # Ensure that GDAL gets a string
        BaseLayer.open(os.fspath(base_layer)),
        time_step, window_length, time_bounds,
    )

def simplify_edge(feature_input, edge):
    """Return the simplified edge corresponding to edge"""
    se = feature_input.road_network.full_edge_to_simplified_edge(edge)
    return features.undirect(se if se != edge else (*edge, 0))

def export_sample_series_segs(feature_input, samples):
    """Given an NxM ndarray of of samples, return a list of CSV-exportable
    rows with the following four fields:
    - timestamp: timestamp, in seconds
    - score: the value taken from samples
    - node1, node2: an edge from the full graph

    The rows are sorted by timestamp ascending, then score descending,
    then node1 and node2 ascending.

    """
    g = feature_input.road_network.network
    simp_edges, time_intervals = features.get_feature_axes(feature_input)
    simp_edge_values = dict(zip(simp_edges, samples))
    edge_list = sorted(g.edges)
    edge_values = [simp_edge_values[simplify_edge(feature_input, e)] for e in edge_list]
    r = []
    for ti, evs in zip(time_intervals, zip(*edge_values)):
        # Take the middle of the interval as the moment of interest
        ts = ti.mean()
        for ev, e in sorted(zip(evs, edge_list), key=lambda ev_e: -ev_e[0]):
            r.append([ts, ev, *e])
    return r

def dump_segs(segs, path):
    """Write the given segments-format list of lists to a file at the
    given path

    """
    with open(path, 'w', newline='') as f:
        csv.writer(f).writerows(segs)

def export_nd_sample_series(feature_input, samples, feature_names=None):
    """Given an NxMxK ndarray of feature values on edges, return a
    JSON-exportable dict with the following data:
    - Key "node_positions": list of node positions like
      {"id": id, "coords": {"lat": lat, "lon": lon}}
    - Key "edges": list of length-two lists of node IDs
    - Key "time_intervals": list of length-two lists of start and stop times
    - Key "value_arrays": list of dicts with the following data:
      - Key "name": string giving a user-readable name for the data array
      - Key "data": length-N list of length-M lists, where N is the
        length of the "edges" list (and corresponds with it) and M is
        the length of the "time_intervals" list (and corresponds with it)

    feature_names, if given, should be a length-K sequence of names.
    Otherwise, it defaults to ["Feature 0", "Feature 1", ...].

    """
    if feature_names is None:
        feature_names = [f"Feature {i}" for i in range(samples.shape[2])]
    g = feature_input.road_network.network
    pos = sorted(zip(g.node, feature_input.road_network.node_position()))
    simp_edges, time_intervals = features.get_feature_axes(feature_input)
    simp_edge_values = dict(zip(simp_edges, samples))
    edge_list = sorted(g.edges)
    edge_values = [simp_edge_values[simplify_edge(feature_input, e)] for e in edge_list]
    edge_values = numpy.moveaxis(edge_values, 2, 0)
    if len(feature_names) != len(edge_values):
        raise ValueError(f"Expected {len(edge_values)} feature names but received {len(feature_names)}")
    return dict(
        node_positions=[dict(id=id_, coords=dict(lon=lon, lat=lat))
                        for id_, (lon, lat) in pos],
        edges=[list(e) for e in edge_list],
        time_intervals=time_intervals.tolist(),
        value_arrays=[dict(name=fn, data=ev) for fn, ev in
                      zip(feature_names, edge_values.tolist())],
    )

def plot_samples(feature_input, values, vmin=None, vmax=None, ax=None):
    """Given a FeatureInput and a length-N ndarray of edge values, plot
    the edges

    """
    import matplotlib.pyplot
    g = feature_input.road_network.network
    imc = feature_input.base_layer.image_coords_from_lon_lat
    pos = (imc(*ll) for ll in feature_input.road_network.node_position())
    pos = dict(zip(g.node, pos))
    simp_edge_values = dict(zip(features.get_feature_axes(feature_input)[0], values))
    edge_list = list(g.edges)
    edge_values = [simp_edge_values[simplify_edge(feature_input, e)] for e in edge_list]
    nx_draw.draw_networkx_edges(
        g, pos, edge_list, edge_color=edge_values,
        edge_cmap=matplotlib.cm.get_cmap('coolwarm'),
        edge_vmin=vmin, edge_vmax=vmax, ax=ax, arrows=False,
    )
    if ax is None: ax = matplotlib.pyplot.gca()
    ax.set_xlim([0, feature_input.base_layer.res_x])
    ax.set_ylim([feature_input.base_layer.res_y, 0])
    ax.set_aspect('auto')

def write_sample_series(feature_input, samples, prefix,
                        vmin=None, vmax=None, v_infer_range=None):
    """Given a FeatureInput and an NxM ndarray of samples, write out a
    series of images to prefix+'000.png' etc.

    If vmin (resp. vmax), it is computed as the v_infer_range[0]
    (resp. ...[1]) fractional percentile of the data, defaulting to
    the min (resp. max) if v_infer_range is None.

    """
    import matplotlib.pyplot as plt
    width = len(str(samples.shape[1] - 1))
    def percentile(p):
        pos = int((samples.size - 1) * p)
        return numpy.partition(samples.reshape(-1), pos)[pos]
    if vmin is None:
        vmin = percentile(0 if v_infer_range is None else v_infer_range[0])
    if vmax is None:
        vmax = percentile(1 if v_infer_range is None else v_infer_range[1])
    for i, arr in enumerate(tqdm.tqdm(samples.T, desc="Visualization images")):
        res_x, res_y = feature_input.base_layer.res_x, feature_input.base_layer.res_y
        height = 8  # This is somewhat arbitrary
        plt.figure(figsize=(res_x * height / res_y, height))
        plot_samples(feature_input, arr, vmin, vmax)
        ax = plt.gca()
        ax.set_position([0, 0, 1, 1])
        ax.set_axis_off()
        plt.savefig(f"{prefix}{i:0{width}}.png", dpi=res_y / height)
        plt.close()

def write_nd_sample_series(feature_input, samples, prefix,
                           vmin=None, vmax=None, v_infer_range=None):
    """Given a FeatureInput and an NxMxK ndarray of samples, write out a
    series of images to prefix+'0_000.png' etc.

    vmin and vmax may each be None (automatic for each component) or a
    length-K sequence.  v_infer_range is None or a single pair that is
    used for all arguments.

    """
    k = samples.shape[2]
    def check(name, x):
        if x is not None and len(x) != k:
            raise ValueError(f"Expected {k} elements in {name}; got {len(x)}")
        return x or [None] * k
    vmins = check('vmin', vmin)
    vmaxes = check('vmax', vmax)
    width = len(str(k - 1))
    for i, (s, vmin, vmax) in enumerate(zip(numpy.moveaxis(samples, 2, 0), vmins, vmaxes)):
        write_sample_series(feature_input, s, f"{prefix}{i:0{width}}_",
                            vmin, vmax, v_infer_range)

def write_image_list(feature_input, images, outpath, anchor=None):
    """With images an iterable of image file names, one per time interval,
    create a file with the corresponding image for each frame at
    outpath.

    If anchor is "start", display images at the start of their window.
    If "end" (the default), display images at the end of their window.

    """
    if anchor is None: anchor = 'end'
    time_intervals = features.get_feature_axes(feature_input)[1]
    time_bounds = TrackSet.time_bounds(feature_input.track_set)
    frame_bounds = TrackSet.frame_bounds(feature_input.track_set)
    if anchor == 'start':
        time_points = numpy.concatenate((time_intervals[:, 0], [time_intervals[-1, 1]]))
    elif anchor == 'end':
        time_points = numpy.concatenate(([time_intervals[0, 0]], time_intervals[:, 1]))
    else:
        raise ValueError("anchor must be either 'start' or 'end'")
    frame_points = numpy.ceil(features.interp(
        time_points, time_bounds[0], frame_bounds[0],
        time_bounds[1], frame_bounds[1],
    )).astype(int)
    with open(outpath, 'w') as f:
        f.writelines(repeat('/dev/null\n', frame_points[0]))
        for flo, fhi, im in zip(frame_points, frame_points[1:], images):
            f.writelines(repeat(im + '\n', fhi - flo))

def standard_images(feature_input, prefix):
    """Return an iterable of file names as generated by
    write_sample_series

    """
    n = len(features.get_feature_axes(feature_input)[1])
    width = len(str(n - 1))
    return (f"{prefix}{i:0{width}}.png" for i in range(n))

def write_standard_image_list(feature_input, prefix, outpath, anchor=None):
    """Like write_image_list, but derive the images from prefix using
    standard_images.

    """
    ims = standard_images(feature_input, prefix)
    write_image_list(feature_input, ims, outpath, anchor)
