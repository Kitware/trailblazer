# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import bisect
import functools
import itertools
import warnings
import numpy as np
import scipy.stats
import tqdm

from trailblazer.feature import config
from trailblazer.traffic.vital import TrackSet

def write_features(cfg, feature_fname, overwrite):
    if os.path.exists(feature_fname) and not overwrite:
        return

    """Extract features given a configuration"""
    config_functions = dict(
        feature_input=config.feature_input,
        road_segment_filter=config.road_segment_filter,
        cluster_features=config.feature_functions,
        vehicle_features=config.feature_functions,
    )
    iconfig = config.apply_parsers(config_functions, cfg['input'])
    fs = concat_features(iconfig["vehicle_features"], iconfig["feature_input"])
    #f_axes = features.get_feature_axes(iconfig["feature_input"])
    #print(f_axes)
    
    np.save(feature_fname, fs)

def concat_samples(sample_arrays):
    """Given an iterable of NxMxKi arrays, return an NxMx∑K array"""
    return np.concatenate([a.reshape((*a.shape[:2], -1)) for a in sample_arrays], axis=-1)

def concat_features(features, feature_input):
    """Call each feature function on the FeatureInput and concatenate the results"""
    return concat_samples(f(feature_input) for f in features)

def linspace_approx_step(start, stop, step):
    """Return a 1D np.ndarray with values from start to stop, including
    both endpoints, uniformly spaced approximately as the given step.
    """
    # XXX It might be nice to pick the rounding direction so as to
    # minimize abs(real_step - step)
    return np.linspace(start, stop, int(round((stop - start) / step + 1)))

def interval_intersect(int1, int2):
    """Given two two-long sequences representing closed intervals, return
    their intersection.  Input or output may be None, indicating an
    empty interval.

    """
    if int1 is not None and int2 is not None:
        mi, ma = max(int1[0], int2[0]), min(int1[1], int2[1])
        if mi <= ma:
            return mi, ma
    return None

def append_if(list, x):
    if x: list.append(x)

def undirect(edge):
    """Normalize edges so that an edge and its reverse compare equal.  For
    multigraph edges, assume that edge indices correspond.

    """
    return (*sorted(edge[:2]), *edge[2:])

def interp(x, x1, y1, x2, y2):
    """Find a point along a line"""
    return ((x2 - x) * y1 + (x - x1) * y2) / (x2 - x1)

class FeatureInput:
    """Simple container for data used to calculate features"""
    __slots__ = (
        'track_set', 'road_mapping', 'road_network', 'base_layer',
        'time_step', 'window_length', 'time_bounds',
    )
    def __init__(
            self, track_set, road_mapping, road_network, base_layer,
            time_step, window_length, time_bounds=None,
    ):
        """Create a FeatureInput from the given TrackSet, RoadMapping,
        RoadNetwork, BaseLayer, time step (in seconds), window length
        (a number of time steps), and time bounds (a pair, in seconds,
        with None indicating the corresponding bound of the TrackSet).

        """
        if time_bounds is None: time_bounds = (None, None)
        for s in self.__slots__:
            setattr(self, s, locals()[s])

def _specified_feature(wrapper, args, kwargs, feature_input):
    """Helper function for feature.  wrapper.func, args, and kwargs
    reflect the original input.

    """
    return wrapper.func(feature_input, *args, **kwargs)

def format_args(func, *args, **kwargs):
    """Render a call to the given function.  Example:
    format_args(print, 'Hello', end=' ') == "print('Hello', end=' ')"

    """
    str_args = ', '.join(itertools.chain(
        map(repr, args), (f'k={v!r}' for k, v in kwargs.items()),
    ))
    return f'{func.__qualname__}({str_args})'

def feature(f):
    """Wrap the given function as a "feature" function.  f should have its
    first argument be the FeatureInput; then this function takes the
    signature from (feature_input, *args, **kwargs) -> result to
    (*args, desc=None, **kwargs) -> feature_input -> result.

    The added desc parameter can be used to provide a description.
    The return values (partially applied functions) of the wrapped
    function store this description as an attribute "desc".

    """
    @functools.wraps(f)
    def wrapper(*args, desc=None, **kwargs):
        r = functools.partial(_specified_feature, wrapper, args, kwargs)
        if desc is None:
            desc = format_args(f, *args, **kwargs)
        r.desc = desc
        return r
    wrapper.func = f
    return wrapper

def get_raw_feature_axes(feature_input):
    """Like get_feature_axes, but return uncombined intervals"""
    net = feature_input.road_network.simplified_network
    edges = sorted(set(map(undirect, net.edges)))
    time_bounds = feature_input.time_bounds
    if None in time_bounds:
        ts_time_bounds = TrackSet.time_bounds(feature_input.track_set)
        time_bounds = [tstb if tb is None else tb
                       for tb, tstb in zip(time_bounds, ts_time_bounds)]
    times = linspace_approx_step(*time_bounds, feature_input.time_step)
    return edges, np.stack([times[:-1], times[1:]], axis=1)

def get_feature_axes(feature_input):
    """Return a pair of a list and an ndarray, the former listing the road
    edges associated with the input in a canonical order and the
    latter listing pairs of start and stop times in increasing order.

    Note that pairs of edges (A, B, X) and (B, A, X) are combined in
    the output.

    Feature computation functions will normally return an array whose
    first two dimensions correspond to these two lists.

    """
    edges, times = get_raw_feature_axes(feature_input)
    wl = feature_input.window_length
    windowed_times = times[:len(times) - wl + 1, 0], times[wl - 1:, 1]
    return edges, np.stack(windowed_times, axis=1)

def get_edge_attrs(feature_input, edge):
    """Return the attribute dictionary associated with edge"""
    edges = feature_input.road_network.simplified_network.edges
    try:
        return edges[edge]
    except KeyError:
        # Reverse edge and try again
        return edges[(edge[1], edge[0], *edge[2:])]

def get_edge_intervals(path):
    """Given a path (a pair of lists of edges and times), return a list of
    pairs of an edge and the time interval on that edge.  Each output
    edge is undirected.

    """
    res = []
    curr_edge = start_time = last_time = None
    for edge, time in zip(*path):
        if edge is not None and len(edge) == 2:
            edge += (0,)
        if edge != curr_edge:
            finish_time = time if last_time is None else (time + last_time) / 2
            if curr_edge is not None:
                res.append((undirect(curr_edge), (start_time, finish_time)))
            curr_edge, start_time = edge, finish_time
        last_time = time
    if curr_edge is not None and start_time != last_time:
        res.append((undirect(curr_edge), (start_time, last_time)))
    return res

def memoize_one(f):
    """Memoize one call of the decorated function by equality of arguments

    """
    key = value = None
    @functools.wraps(f)
    def wrapper(*args, **kwargs):
        nonlocal key, value
        key_ = args, kwargs
        if key != key_:
            key, value = key_, f(*args, **kwargs)
        return value
    return wrapper

@memoize_one
def get_all_edge_intervals(paths):
    """Given a dictionary whose values are arguments to
    get_edge_intervals, return a new dictionary mapping edges to lists
    of pairs of the original keys and the intervals for that edge from
    get_edge_intervals

    """
    r = {}
    for k, path in paths.items():
        for e, ti in get_edge_intervals(path):
            r.setdefault(e, []).append((k, ti))
    return r

@memoize_one
def get_all_meter_coordinates(feature_input):
    """Given a FeatureInput, return a dictionary mapping pairs of track ID
    and timestamp to a two-long ndarray of meter coordinates

    """
    meters = feature_input.base_layer.meters_from_lon_lat
    return {(tid, d.timestamp): np.array(meters(d.lon, d.lat))
            for tid, track in feature_input.track_set.items()
            for d in track.detections}

@memoize_one
def get_all_node_meter_coordinates(feature_input):
    """Given a FeatureInput, return a dictionary mapping node IDs to
    two-long ndarrays of meter coordinates.

    """
    meters = feature_input.base_layer.meters_from_lon_lat
    node_position = feature_input.road_network.node_position
    return {
        nid: np.array(meters(*node_position([nid])[0]))
        for nid in feature_input.road_network.simplified_network.node
    }

def clip_around_detections(detections, time_interval):
    """Return the narrowest sublist of detections (a list of Detections)
    containing time_interval.

    """
    times = [d.timestamp for d in detections]
    lo = bisect.bisect(times, time_interval[0]) - 1
    hi = bisect.bisect_left(times, time_interval[1]) + 1
    return detections[lo:hi]

def clip_around_detections_instant(detections, time):
    """Return the narrowest sublist of detections (a list of Detections)
    containing time.  If there is a detection at exactly time, try to
    expand in either direction.

    """
    times = [d.timestamp for d in detections]
    lo = max(bisect.bisect_left(times, time) - 1, 0)
    hi = min(bisect.bisect(times, time) + 1, len(times))
    return detections[lo:hi]

def intervals_exceeding_distance(distance_times, minimum_distance):
    """Given a list of distance--time pairs, return a list of intervals
    where the distance is at least minimum_distance.

    """
    result = []
    start_time = prev_time = prev_dist = None
    for dist, time in distance_times:
        dist_high = dist >= minimum_distance
        prev_dist_high = prev_dist is not None and prev_dist >= minimum_distance
        if dist_high != prev_dist_high:
            thresh_time = time if prev_dist is None else interp(
                minimum_distance, prev_dist, prev_time, dist, time,
            )
            if dist_high:
                start_time = thresh_time
            else:
                result.append((start_time, thresh_time))
        prev_dist, prev_time = dist, time
    if prev_dist is not None and dist_high:
        result.append((start_time, time))
    return result

@memoize_one
def get_all_distant_edge_intervals(
        feature_input, intersection_exclusion_radius,
):
    """Return a dictionary mapping edges to dictionaries mapping track IDs
    to intervals that the track is on the edge and at least the given
    distance (intersection_exclusion_radius, in meters) away from the
    node at either end of the edge.

    """
    ii = interval_intersect
    paths = feature_input.road_mapping.simplified_paths
    all_edge_intervals = get_all_edge_intervals(paths)
    all_meter_coordinates = get_all_meter_coordinates(feature_input)
    all_node_meters_coordinates = get_all_node_meter_coordinates(feature_input)
    new_all_edge_intervals = {}
    for edge, edge_intervals in all_edge_intervals.items():
        intersections = np.stack([all_node_meters_coordinates[nid] for nid in edge[:2]])
        def intersection_distance(tid, d):
            """Shortest distance of detection from either end of this edge"""
            m = all_meter_coordinates[tid, d.timestamp]
            return ((m - intersections) ** 2).sum(1).min() ** .5
        new_intervals = {}
        for tid, ti in edge_intervals:
            r = []
            dets = clip_around_detections(feature_input.track_set[tid].detections, ti)
            if len(dets) < 2: continue # Might not be necessary
            distance_times = [(intersection_distance(tid, d), d.timestamp) for d in dets]
            # XXX Will it really ever be the case that a track enters and
            # then exits the exclusion zone?
            good_intervals = intervals_exceeding_distance(distance_times, intersection_exclusion_radius)
            if good_intervals:
                append_if(r, ii(good_intervals[0], ti))
                r.extend(good_intervals[1:-1])
                if len(good_intervals) >= 2:
                    append_if(r, ii(good_intervals[-1], ti))
            if r:
                new_intervals.setdefault(tid, []).extend(r)
        new_all_edge_intervals[edge] = new_intervals
    return new_all_edge_intervals

@feature
def segment_activity(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius
    (measured in meters), return a 1D numpy array of road segment
    activities, with each entry corresponding to a road edge as
    returned by get_feature_axes.

    If 2 * intersection_exclusion_radius >= edge_length, return 0 for
    that segment.

    """
    edges = get_raw_feature_axes(feature_input)[0]
    adei = get_all_distant_edge_intervals(feature_input, intersection_exclusion_radius)
    def activity(e):
        return sum(tih - til for tis in adei.get(e, {}).values() for til, tih in tis)
    return np.array([activity(e) for e in edges])

def intervals_on_edge(
        feature_input, intersection_exclusion_radius, edge, time,
):
    """Compute the intervals that each track appears on the given edge in
    the given time interval at least the given distance away
    (intersection_exclusion_radius, in meters) from each node.

    Return a dictionary mapping track IDs to a list of intervals.

    """
    ii = interval_intersect
    all_distant_edge_intervals = get_all_distant_edge_intervals(feature_input, intersection_exclusion_radius)
    edge_intervals = {}
    for tid, intervals in all_distant_edge_intervals.get(edge, {}).items():
        r = [ii(ti, time) for ti in intervals if ii(ti, time)]
        if r:
            edge_intervals[tid] = r
    return edge_intervals

def windowed_sum(samples, window_length):
    """Given an NxMx... array, return an Nx(M-L+1)x... array, where L is
    window_length, by adding with length-L windows.

    """
    axis = 1
    samples = np.moveaxis(samples, axis, 0)
    zeros = np.broadcast_to(0, (1,) + samples.shape[1:])
    cumsum = np.concatenate((zeros, np.cumsum(samples, axis=0)))
    result = cumsum[window_length:] - cumsum[:-window_length]
    return np.moveaxis(result, 0, axis)

def on_feature_axes(feature_input, f, default=None):
    """Evaluate f (signature: (feature_input, edge, time, /) -> (result, weight))
    on each edge--time pair from get_raw_feature_axes(feature_input),
    returning an ndarray with axes corresponding to
    get_feature_axes(feature_input).

    If the sum of the weights is 0, return default (default 0)

    """
    if default is None: default = 0.
    wl = feature_input.window_length
    edges, times = get_raw_feature_axes(feature_input)
    all_values = [[f(feature_input, edge, time) for time in times]
                  for edge in tqdm.tqdm(edges, desc="Feature on edges")]
    values, weights = np.moveaxis(all_values, 2, 0)
    cum_weights = windowed_sum(weights, wl)
    return np.where(cum_weights, windowed_sum(values * weights, wl) / cum_weights, default)

def _vehicle_density_at(
        intersection_exclusion_radius, feature_input, edge, time,
):
    """Compute the vehicle density for a specific edge and time
    interval.

    edge should be normalized as by undirect.

    """
    edge_length = get_edge_attrs(feature_input, edge)['length']
    edge_length -= 2 * intersection_exclusion_radius
    if edge_length <= 0:
        return 0., 1
    intervals = intervals_on_edge(feature_input, intersection_exclusion_radius, edge, time)
    if not intervals:
        return 0., 1
    all_intervals = np.concatenate(list(intervals.values()))
    return -np.subtract(*all_intervals.T).sum() / (time[1] - time[0]) / edge_length, 1

@feature
def vehicle_density(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius
    (measured in meters), return a 2D numpy array of vehicle
    densities, with each row corresponding to an road edge and each
    column to a time window as returned by get_feature_axes.

    If 2 * intersection_exclusion_radius >= edge_length, return 0 for
    that edge's row.

    """
    return on_feature_axes(feature_input, functools.partial(
        _vehicle_density_at, intersection_exclusion_radius,
    ))

@memoize_one
def get_all_distant_starts(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius,
    return a dictionary mapping edges to sorted ndarrays of when
    tracks appear on the edge.

    """
    all_intervals = get_all_distant_edge_intervals(feature_input, intersection_exclusion_radius)
    return {edge: np.sort([start for ivs in intervals.values() for start, _ in ivs])
            for edge, intervals in all_intervals.items()}

def _time_between_vehicles_at(
        intersection_exclusion_radius, feature_input, edge, time,
):
    """Compute the average time between vehicles for a specific edge and time interval.

    Edge should be normalized as by undirect.

    """
    starts = get_all_distant_starts(feature_input, intersection_exclusion_radius)
    starts = starts.get(edge, ())
    if len(starts) < 2:
        return 0., 1
    # Clip around the interval
    lo = max(np.searchsorted(starts, time[0], 'right') - 1, 0)
    hi = min(np.searchsorted(starts, time[1], 'right') + 1, len(starts))
    starts = starts[lo:hi]
    if len(starts) < 2:
        return 0., 0
    weights = np.ones((len(starts) - 1,))
    if starts[0] <= time[0]:
        if starts[1] > time[1]:
            assert len(weights) == 1
            weights[0] = (time[1] - time[0]) / (starts[1] - starts[0])
        else:
            weights[0] = (starts[1] - time[0]) / (starts[1] - starts[0])
    elif starts[-1] > time[1]:
        weights[-1] = (time[1] - starts[-2]) / (starts[-1] - starts[-2])
    weight = weights.sum()
    return (np.diff(starts) * weights).sum() / weight, weight

# XXX Zero is a horrendous "no data" return value
@feature
def time_between_vehicles(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius
    (measured in meters), return a 2D numpy array of average time
    between vehicles, with each row corresponding to an road edge and
    each column to a time window as returned by get_feature_axes.

    If there are no observations, return 0 for that edge and time
    interval.

    """
    return on_feature_axes(feature_input, functools.partial(
        _time_between_vehicles_at, intersection_exclusion_radius,
    ))

def _vehicle_rate_at(
        intersection_exclusion_radius, feature_input, edge, time,
):
    """Compute the average vehicle rate for a specific edge and time interval.

    Edge should be normalized as by undirect.

    """
    starts = get_all_distant_starts(feature_input, intersection_exclusion_radius)
    try:
        starts = starts[edge]
    except KeyError:
        return 0., 1
    edge_length = get_edge_attrs(feature_input, edge)['length']
    edge_length -= 2 * intersection_exclusion_radius
    if edge_length <= 0:
        return 0., 1
    lo = np.searchsorted(starts, time[0], 'right')
    hi = np.searchsorted(starts, time[1], 'right')
    return (hi - lo) / (time[1] - time[0]) / edge_length, 1

@feature
def vehicle_rate(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius
    (measured in meters), return a 2D numpy array of average vehicle
    rate, normalized by road length (new tracks per second per meter),
    with each row corresponding to an road edge and each column to a
    time window as returned by get_feature_axes.

    If 2 * intersection_exclusion_radius >= edge_length, return 0 for
    that edge's row.

    """
    return on_feature_axes(feature_input, functools.partial(
        _vehicle_rate_at, intersection_exclusion_radius,
    ))

def _vehicle_speed_at(
        intersection_exclusion_radius, feature_input, edge, time,
):
    """Compute the average vehicle speed for a specific edge and time interval.

    Edge should be normalized as by undirect.

    """
    intervals = intervals_on_edge(feature_input, intersection_exclusion_radius, edge, time)
    if not intervals:
        return 0., 0.
    all_meter_coordinates = get_all_meter_coordinates(feature_input)
    dists, times = 0, 0
    for tid, ivs in intervals.items():
        all_dets = feature_input.track_set[tid].detections
        for ti in ivs:
            if ti[0] == ti[1]:
                continue
            dets = clip_around_detections(all_dets, ti)
            if len(dets) < 2: continue
            times_pos = [(d.timestamp, all_meter_coordinates[tid, d.timestamp]) for d in dets]
            times_pos[0], times_pos[-1] = (
                (ti[0], interp(ti[0], *times_pos[0], *times_pos[1])),
                (ti[-1], interp(ti[-1], *times_pos[-1], *times_pos[-2])),
            )
            delta_pos = np.diff(np.stack([pos for _, pos in times_pos]), axis=0)
            dists += ((delta_pos ** 2).sum(1) ** .5).sum()
            times += ti[1] - ti[0]
    return dists / times if times else 0., times

@feature
def vehicle_speed(feature_input, intersection_exclusion_radius):
    """With the given FeatureInput and intersection exclusion radius
    (measured in meters), return a 2D numpy array of average vehicle
    speed, with each row corresponding to an road edge and each column
    to a time window as returned by get_feature_axes.

    If there are no observations, return 0 for that edge and time
    interval.

    """
    return on_feature_axes(feature_input, functools.partial(
        _vehicle_speed_at, intersection_exclusion_radius,
    ))

def _vehicle_local_density_at(
        intersection_exclusion_radius, kernel_size, resample_step,
        feature_input, edge, time,
):
    """Compute the average vehicle local density for a specific edge and
    time interval.

    Edge should be normalized as by undirect.

    """
    intervals = intervals_on_edge(feature_input, intersection_exclusion_radius, edge, time)
    if not intervals:
        return 0., 1  # Get the same result as the code below, but faster
    resample_times = linspace_approx_step(*time, resample_step)
    resample_times = (resample_times[:-1] + resample_times[1:]) / 2
    bins = [[] for _ in range(len(resample_times))]
    for tid, tis in intervals.items():
        for ti in tis:
            start, stop = interp(ti, time[0], -0.5, time[1], len(bins) - 0.5)
            for b in bins[int(np.ceil(start)):int(np.floor(stop)) + 1]:
                b.append(tid)
    all_meter_coordinates = get_all_meter_coordinates(feature_input)
    norm, vonmises = scipy.stats.norm.pdf, scipy.stats.vonmises.pdf
    scale, kappa = kernel_size[0], (kernel_size[1] * np.pi / 180) ** -2
    scores = []
    for t, b in zip(resample_times, bins):
        if not b:
            scores.append(0)
            continue
        pos, headings = [], []
        for tid in b:
            all_dets = feature_input.track_set[tid].detections
            dets = clip_around_detections_instant(all_dets, t)
            if len(dets) < 2: continue
            assert 2 <= len(dets) <= 3
            posses = [all_meter_coordinates[tid, d.timestamp] for d in dets]
            if len(dets) == 2:
                times = [d.timestamp for d in dets]
                pos.append(interp(t, times[0], posses[0], times[1], posses[1]))
            else:
                pos.append(posses[1])
            headings.append(np.arctan2(*(posses[-1] - posses[0])))
        if not pos:
            continue
        pos, headings = np.stack(pos), np.stack(headings)
        score = 0
        for p, h in zip(pos, headings):
            score += (
                norm(pos[:, 0], p[0], scale) *
                norm(pos[:, 1], p[1], scale) *
                vonmises(headings, kappa, h)
            ).sum()
        scores.append(score / len(pos))
    return sum(scores) / len(bins), 1

@feature
def vehicle_local_density(
        feature_input, intersection_exclusion_radius, kernel_size,
        resample_step,
):
    """With the given FeatureInput, intersection exclusion radius
    (measured in meters), kernel size (a pair, the first measured in
    meters and the second in degrees), and the resampling step
    (measured in seconds), return a 2D numpy array of average vehicle
    local density, with each row corresponding to a road edge and each
    column to a time window as returned by get_feature_axes.

    If there are no observations, return 0 for that edge and time
    interval.

    """
    return on_feature_axes(feature_input, functools.partial(
        _vehicle_local_density_at,
        intersection_exclusion_radius, kernel_size, resample_step,
    ))

def _vehicle_size_frac_at(
        intersection_exclusion_radius, size_threshold, side,
        feature_input, edge, time,
):
    """Compute the average fraction of vehicles of the given size range
    for a specific edge and time interval.

    Edge should be normalized as by undirect.

    """
    intervals = intervals_on_edge(feature_input, intersection_exclusion_radius, edge, time)
    def cat(det):
        return 'high' if det.area >= size_threshold else 'low'
    ii = interval_intersect
    times = {'high': 0, 'low': 0}
    for tid, ivs in intervals.items():
        all_dets = feature_input.track_set[tid].detections
        for ti in ivs:
            # XXX So many edge cases; I won't handle them all
            if ti[0] == ti[1]:
                continue
            # Handle category changes with nearest-neighbors
            dets = clip_around_detections(all_dets, ti)
            assert len(dets) >= 2
            times_cat = [(d.timestamp, cat(d)) for d in dets]
            for (t1, c1), (t2, c2) in zip(times_cat, times_cat[1:]):
                tmid = (t1 + t2) / 2
                ti1 = ii((t1, tmid), ti)
                if ti1:
                    times[c1] += ti1[1] - ti1[0]
                ti2 = ii((tmid, t2), ti)
                if ti2:
                    times[c2] += ti2[1] - ti2[0]
    total = sum(times.values())
    return times[side] / total if total else 0, total

@feature
def vehicle_size_frac(
        feature_input, intersection_exclusion_radius,
        size_threshold, side,
):
    """With the given FeatureInput, intersection exclusion radius
    (measured in meters), size threshold (measured in square meters),
    and side (one of the strings "high" or "low"), return a 2D numpy
    array of the average fraction of vehicles whose size is on the
    given side of the threshold, with each row corresponding to a road
    edge and each column to a time window as returned by
    get_feature_axes.

    If there are no observations, return 0.5 for that edge and time
    interval.

    """
    return on_feature_axes(feature_input, functools.partial(
        _vehicle_size_frac_at,
        intersection_exclusion_radius, size_threshold, side,
    ), default=0.5)

def _log_wrapper(feature, zero, expect_zero, feature_input):
    """Helper for log.  Returns the natural log of the original values."""
    samples = feature(feature_input)
    # Replace 0s with 1s to work around the warning about computing
    # log(0), actually returning which is avoided below
    log = np.log(np.where(samples, samples, 1))
    if expect_zero == 'high' and (zero <= log[samples != 0]).any():
        warnings.warn("Expected zero high, but as large values in output")
    elif expect_zero == 'low' and (zero >= log[samples != 0]).any():
        warnings.warn("Expected zero low, but as small values in output")
    return np.where(samples, log, zero)

def log(feature, zero, expect_zero=None, desc=None):
    """Transform a feature, returning the natural log of its values.  Use
    the specified value for originally zero values.

    If expect_zero is not None (the default), it should be one of the
    strings "high" or "low".  If the zero value is not in the
    indicated relation to all the mapped non-zero values, issue a
    warning.

    """
    expect_zero_options = None, 'high', 'low'
    if expect_zero not in expect_zero_options:
        raise ValueError(f"expect_zero should be one of {expect_zero_options} but is {expect_zero!r}")
    r = functools.partial(_log_wrapper, feature, zero, expect_zero)
    if desc is None:
        desc = format_args(log, f'{feature.desc}', zero, expect_zero)
    r.desc = desc
    return r
