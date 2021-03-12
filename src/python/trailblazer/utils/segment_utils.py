import matplotlib.pyplot as plt
import os
import pathlib

import networkx as nx


def load_segments_file(segs_filepath):
    segs = []
    with open(segs_filepath, 'r') as f:
        for line in f:
            time, conf, node_a, node_b = map(str.strip, line.split(','))

            conf = None if conf == "None" else float(conf)
            segs.append((node_a, node_b, conf))

    return segs


def visualize_segments(outdir,
                       filename,
                       road_network,
                       segments_filepath,
                       zoom_to_track=True,
                       threshold=0.75):
    """ Plots segments (road edges) onto the road network

    :param segments: Set of segments along with a confidence value to
    visualize; as a list of triples (start_id, end_id, confidence)

    :param zoom_to_track: If True, zoom the plot to the road
    mapping for the given track

    :param threshold: If None, change color intensity based on
    normalized segment confidence; otherwise, visualize only segments
    with confidence at or above the given threshold (with a fixed
    intensity)

    Generates one plot file per track.
    """
    if not os.path.isdir(outdir):
        pathlib.Path(outdir).mkdir(parents=True, exist_ok=True)

    segments = load_segments_file(segments_filepath)

    aspect_ratio = (road_network._base_layer.res_x /
                    road_network._base_layer.res_y)

    G = road_network.network
    node_pos = dict(zip(G.node.keys(),
                        [(G.node[x]['easting'], G.node[x]['northing'])
                         for x in G.node.keys()]))

    plt.clf()
    plt.cla()
    fgr = plt.figure(dpi=240)

    nx.draw_networkx_edges(G, node_pos, arrows=False)

    def normalize_score(min_score, max_score, score):
        d_score = max_score - min_score

        return (score - min_score) / d_score

    segments_to_draw = {}
    segments_to_draw_count = {}
    min_conf, max_conf = float('inf'), -float('inf')
    for segment in segments:
        start_id, end_id, conf = segment
        k = (int(start_id), int(end_id))

        if conf is not None:
            min_conf = min(min_conf, conf)
            max_conf = max(max_conf, conf)

            if threshold is None:
                segments_to_draw[k] = max(
                    segments_to_draw.get(k, -float('inf')),
                    conf)
                if k not in segments_to_draw_count:
                    segments_to_draw_count[k] = 0
                segments_to_draw_count[k] += 1
            elif conf >= threshold:
                segments_to_draw[k] = max(
                    segments_to_draw.get(k, -float('inf')),
                    conf)
                if k not in segments_to_draw_count:
                    segments_to_draw_count[k] = 0
                segments_to_draw_count[k] += 1
        else:
            segments_to_draw[k] = None

    # Filter out segments with a low number of conf values
    for s,cnt in segments_to_draw_count.items():
        if cnt < 20:
            del segments_to_draw[s]

    # Plot segments
    min_x, max_x, min_y, max_y = (float('inf'),
                                  -float('inf'),
                                  float('inf'),
                                  -float('inf'))

    if segments_to_draw:
        edges = []
        colors = []
        for (start_id, end_id), conf in segments_to_draw.items():
            x0, y0 = node_pos[start_id]
            x1, y1 = node_pos[end_id]
            min_x = min(min_x, x0, x1)
            max_x = max(max_x, x0, x1)
            min_y = min(min_y, y0, y1)
            max_y = max(max_y, y0, y1)

            if conf is None or threshold is not None:
                color = (1, 0, 0)
            else:
                color = (normalize_score(min_conf, max_conf, conf), 0, 0)

            edges.append((start_id, end_id))
            colors.append(color)

        nx.draw_networkx_edges(G,
                               node_pos,
                               edgelist=edges,
                               edge_color=colors,
                               arrows=False)

        if not zoom_to_track:
            min_x, max_x, max_y, min_y = road_network._base_layer.extent_meters

        w = max_x - min_x
        h = max_y - min_y
        x_pad = max(((aspect_ratio * h * 1.1) - w) / 2, w * 0.1)
        y_pad = max((((w * 1.1) / aspect_ratio) - h) / 2, h * 0.1)

        plt.xlim(min_x - x_pad, max_x + x_pad)
        plt.ylim(min_y - y_pad, max_y + y_pad)

    plt.xlabel('Easting (meters)', fontsize=12)
    plt.ylabel('Northing (meters)', fontsize=12)
    plt.savefig(os.path.join(outdir, filename+".png"))
    plt.close(fgr)
