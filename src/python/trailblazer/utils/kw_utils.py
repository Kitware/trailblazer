import os
import cv2
import json
import ntpath
import numpy as np
from scipy.spatial import cKDTree

from trailblazer.traffic.vital import Detection
from trailblazer.traffic.vital import Track
from trailblazer.traffic.vital import TrackSet
from trailblazer.traffic.vital import Event
from trailblazer.traffic.base_layer import BaseLayer
from trailblazer.traffic.road_network import RoadNetwork
from trailblazer.utils import path_utils as path_utils, pixel_utils as px

"""
** kw18 format ***
      Index   Metadata
       0      Track-id
       1      Track-length (# of detections)
       2      Frame-number (-1 if not available)
       3-4    Tracking-plane-loc(x,y) (Could be same as World-loc)
       5-6    Velocity(x,y)
       7-8    Image-loc(x,y)
       9-12   Img-bbox(top-left x, top-left y, bottom-rightB x, bottom-right y)
       13     Area (0 - when not available)
       14-16  World-loc(x,y,z) (longitude, latitude, 0 when not available)
       17     Timestamp(-1 if not available)
       18     Track-confidence(-1_when_not_available)
"""

def read_kw18(kw18_file):
    track_set = {}
    with open(kw18_file) as kw18:
        for line in kw18:
            if line.startswith('#'):
                continue
            columns = line.split(' ')
            detection = Detection()
            track_id = int(columns[0])
            detection.frame_number = int(columns[2])
            detection.tracking_plane_loc_x = float(columns[3])
            detection.tracking_plane_loc_y = float(columns[4])
            detection.velocity_x = float(columns[5])
            detection.velocity_y = float(columns[6])
            detection.image_loc_x = float(columns[7])
            detection.image_loc_y = float(columns[8])
            detection.image_bbox_TL_x = float(columns[9])
            detection.image_bbox_TL_y = float(columns[10])
            detection.image_bbox_BR_x = float(columns[11])
            detection.image_bbox_BR_y = float(columns[12])
            detection.area = float(columns[13])
            detection.lon = float(columns[14])
            detection.lat = float(columns[15])
            if detection.lat < -60 or detection.lat > 60:
                detection.lon = float(columns[15])
                detection.lat = float(columns[14])
            detection.alt = float(columns[16])
            detection.timestamp = float(columns[17])

            if track_id not in track_set.keys():
                track = Track()
                track.id = track_id
                track_set[track_id] = track
            else:
                track = track_set[track_id]

            track.detections.append(detection)

    for tid,track in track_set.items():
        track.compute_bounds()
    return track_set


def write_kw18(track_set, kw18_file):

    kw18_header_string = '# 1:Track-id  2:Track-length  3:Frame-number' + \
                         '  4-5:Tracking-plane-loc(x,y)  6-7:velocity(x,y)' + \
                         '  8-9:Image-loc(x,y)' + \
                         '  10-13:Img-bbox(TL_x,TL_y,BR_x,BR_y)' + \
                         '  14:Area  15-17:World-loc(x,y,z) 18:timestamp' + \
                         '  19:object-type-id  20:activity-type-id\n'

    kw18 = open(kw18_file, 'w')
    kw18.write(kw18_header_string)
    for tid, track in track_set.items():
        for detection in track.detections:
            kw18.write(str(track.id) + ' ' +
                       str(len(track.detections)) + ' ' +
                       str(detection.frame_number) + ' ' +
                       str(detection.tracking_plane_loc_x) + ' ' +
                       str(detection.tracking_plane_loc_y) + ' ' +
                       str(detection.velocity_x) + ' ' +
                       str(detection.velocity_y) + ' ' +
                       str(detection.image_loc_x) + ' ' +
                       str(detection.image_loc_y) + ' ' +
                       str(detection.image_bbox_TL_x) + ' ' +
                       str(detection.image_bbox_TL_y) + ' ' +
                       str(detection.image_bbox_BR_x) + ' ' +
                       str(detection.image_bbox_BR_y) + ' ' +
                       str(detection.area) + ' ' +
                       str(detection.lon) + ' ' +
                       str(detection.lat) + ' ' +
                       str(detection.alt) + ' ' +
                       str(detection.timestamp) + ' -1\n'
                       )

def read_pNeuma(csv_file):
    line_cnt = 0
    track_set = {}
    with open(csv_file) as csv:
        for line in csv:
            line_cnt = line_cnt + 1
            if line.startswith('#') or line_cnt == 1:
                continue
            columns = line.split(';')

            track_id = int(columns[0])
            #if track_id != 8:
            #    continue

            type = columns[1]
            travel_dist = float(columns[2])
            avg_speed = float(columns[3])

            if track_id not in track_set.keys():
                track = Track()
                track.id = track_id
                track_set[track_id] = track
            else:
                track = track_set[track_id]

            print("Processing track "+str(track_id))



            movement_start = False
            for d in range(4,len(columns)-1,6):
                if not movement_start and float(columns[d+3]) == 0 and float(columns[d+4]) == 0:
                    continue
                movement_start = True
                detection = Detection()
                detection.timestamp = float(columns[d + 5])
                detection.frame_number = int(round(detection.timestamp / 0.04))
                # columns[d+2] is speed
                detection.tracking_plane_loc_x = 0
                detection.tracking_plane_loc_y = 0
                detection.velocity_x = float(columns[d+3])
                detection.velocity_y = float(columns[d+4])
                detection.image_loc_x = 0
                detection.image_loc_y = 0
                detection.image_bbox_TL_x = 0
                detection.image_bbox_TL_y = 0
                detection.image_bbox_BR_x = 0
                detection.image_bbox_BR_y = 0
                detection.area = 0
                detection.lon = float(columns[d+1])
                detection.lat = float(columns[d])
                detection.alt = 0


                track.detections.append(detection)

            track.compute_bounds()

    return track_set

def kw18_to_json(kw18_fname, json_fname):
    with open(kw18_fname) as f:
        data = [line.split() for line in f.readlines()[1:]]
    data = [[int(m) for m in l[:3]] + [float(m) for m in l[3:]] for l in data]
    col = ['id', 'len', 'frame', 'tx', 'ty', 'vx', 'vy', 'ix', 'iy',
           'left', 'top', 'right', 'bottom', 'area', 'x', 'y', 'alt', 't', 'conf']
    jdata = [dict(zip(col, l)) for l in data]
    with open(json_fname, 'wt') as f:
        json.dump(jdata, f)

def write_kwe(event_set, kwe_file):
    with open(kwe_file, 'w') as of:
        for eId, e in event_set.items():
            if len(e.tracks) == 0:
                continue
            e.compute_bounds()
            of.write('0 ' +  # Step (??)
                     str(e.type) + ' ' +
                     str(e.id) + ' ' +
                     str(e.start_time) + ' ' +  # Start time
                     str(e.start_frame) + ' ' +  # Start frame
                     str(e.end_time) + ' ' +  # Stop time
                     str(e.end_frame) + ' ' +  # Stop frame
                     '1.0 ' +  # Probability
                     str(e.min_x) + ' ' +  # Bounding box min x
                     str(e.min_y) + ' ' +  # Bounding box min y
                     str(e.max_x) + ' ' +  # Bounding box max x
                     str(e.max_y) + ' ' +  # Bounding box max y
                     str(len(e.tracks))  # Number of tracks in event
                     )
            for track in e.tracks:
                of.write(str(' ' + str(track.id)))  # track id's in the event
            for track in e.tracks:
                of.write(' ' + str(track.start_time) +
                         ' ' + str(track.start_frame) +
                         ' ' + str(track.end_time) +
                         ' ' + str(track.end_frame)
                         )

def create_vpView(track_set, out_dir, osm_fname=None, base_layer_fname=None):

    if base_layer_fname is not None:
        base_layer = BaseLayer.open(base_layer_fname)
    else:
        if osm_fname is not None:
            print('Creating base layer from osm file...')
            road_network = RoadNetwork.from_osm(osm_fname)
            base_layer = BaseLayer.from_road_network(road_network)
        else:
            print('Creating image base layer from tracks...')
            base_layer = BaseLayer.from_track_list(track_set)
        base_layer_fname = out_dir + '/base_layer.tif'
        print('Writing out the base image...' + base_layer_fname)
        base_layer.save_geotiff(base_layer_fname)

    # Write new image coordinates to each track detection
    print("Converting world coordinates to image coordinates")
    px.world_to_image_coordinates(base_layer, track_set)

    write_vpView_files(track_set, out_dir, base_layer_fname)


def write_vpView_files(track_set, out_dir, base_layer):

    if isinstance(base_layer,BaseLayer):
        print('Writing out the base image...')
        base_layer_fname = os.path.abspath(out_dir + '/base_layer.tif').replace('\\', '/')
        base_layer.save_geotiff(base_layer_fname)
    else:# Should be string location of the base_layer file
        base_layer_fname = os.path.abspath(base_layer).replace('\\', '/')

    base_layer_cmpts = os.path.splitext(base_layer_fname)
    vpView_base_layer_fname = base_layer_cmpts[0] + base_layer_cmpts[1]

    out_dir = os.path.abspath(out_dir).replace('\\', '/')
    path_utils.clean_directory(out_dir)
    # Relative names for use in the vpView image list (either the original mask or the recolored version)
    vpView_base_layer_rname = os.path.relpath(vpView_base_layer_fname, out_dir).replace('\\', '/')

    # Write out a new base_layer with a different color scheme
    #recolor_base_layer(base_layer_fname, vpView_base_layer_fname,
    #                   bg_color=(255, 255, 255),
    #                   road_color=(0, 0, 0),
    #                   build_low_color=(32, 0, 0),
    #                   build_high_color=(255, 0, 0))

    print('Creating a image list file referencing the base image...')
    imlist_fname = out_dir + '/image_list.txt'
    min_frame, max_frame = TrackSet.frame_bounds(track_set)
    with open(imlist_fname, 'w') as f:
        for _ in range(max_frame + 1):
            f.write(vpView_base_layer_rname+'\n')
    print('Writing out the new kw18 file with image coordinates...')
    out_kw18_fname = out_dir + '/tracks.kw18'
    write_kw18(track_set, out_kw18_fname)
    print('Writing out the vpView project file...')
    with open(out_dir + '/vpview.prj', 'w') as f:
        f.write('DataSetSpecifier=./' + ntpath.basename(imlist_fname) + '\n')
        f.write('TracksFile=./' + ntpath.basename(out_kw18_fname) + '\n')

    print('Finished!')

def recolor_base_layer(input, output, bg_color, road_color, build_low_color, build_high_color):
    im = cv2.imread(input)
    im = im[..., ::-1]
    indexed = im[..., 2].astype(np.int16) - (im[..., 0] != 0)
    build_colors = np.stack([np.linspace(lo, hi, 256) for lo, hi in
                                zip(build_low_color, build_high_color)], axis=-1)[1:]
    palette = np.concatenate(([bg_color], build_colors, [road_color])).round()
    assert ((0 <= palette) & (palette < 256)).all()
    im_out = palette.astype(np.uint8)[indexed]
    cv2.imwrite(output, im_out)

# The point of this algorithm is to preserve the original track during the downsample
# This does copy the detections, ds_track_set will have new copies of the detections
# This does no kinematic interpolation between times
# This works best if the kw18 detections are in a consistently sampled time interval
def downsample_track_set(track_set, rate_Hz):
    ds_track_set = {}
    sample_s = 1 / rate_Hz
    for tid, track in track_set.items():
        # Make a new track to downsample to
        ds_track = Track()
        ds_track.id = tid
        ds_track_set[tid] = ds_track

        # Walk the detections in time, keeping detections that occur
        # at our requested rate
        for det in track.detections:
            if det.timestamp % sample_s == 0:
                # TODO check the time difference and interpolate if the track time is not on the time step
                ds_det = det.copy()
                ds_det.frame_number = int(round(ds_det.timestamp/sample_s))
                ds_track.detections.append(ds_det)

        ds_track.compute_bounds()

    return ds_track_set


# This algorithm randomly drops detections from a track given a
# probability of detection.  Modifies the track_set in place and
# returns it as a convenience
def pd_track_set(track_set, probability_of_detection):
    for tid, track in track_set.items():
        track.detections = list(np.array(track.detections)
                                [np.random.rand(len(track.detections))
                                 <= probability_of_detection])

        track.compute_bounds()

    return track_set


# Generate false alarm detections based on a false alarm rate for
# a given area and time
# false detections per km^2 per minute
def generate_false_alarms(time_bounds, frame_bounds, base_layer, false_alarm_rate):
    ret = base_layer.extent_meters
    area = (ret[1]-ret[0])*(ret[3]-ret[2])/1e6   # km^2

    # Bounds of the detection (meters)
    bb_w, bb_h = 2, 2
    gsd = base_layer.gsd[0]

    nframes = frame_bounds[1] - frame_bounds[0] + 1
    times = np.linspace(time_bounds[0], time_bounds[1], nframes)
    interval = (time_bounds[1] - time_bounds[0]) * (nframes / (nframes - 1))
    false_dets = false_alarm_rate*area*interval/60
    false_dets = np.random.poisson(lam=false_dets)
    out_dets = []
    for i, _ in enumerate(range(false_dets)):
        ind = np.random.randint((frame_bounds[1] - frame_bounds[0]) + 1)
        frame_number = frame_bounds[0] + ind
        timestamp = times[ind]
        xc = np.random.rand()*base_layer.res_x
        yc = np.random.rand()*base_layer.res_y
        bbox = [xc - (bb_w / gsd),
                yc + (bb_h / gsd),
                xc + (bb_w / gsd),
                yc - (bb_h / gsd)]
        area = bb_w*bb_h
        lon_lat = base_layer.lon_lat_from_image_coords([xc, yc])
        world_xc, world_yc = base_layer.meters_from_image_coords([xc, yc])

        det = Detection()
        det.frame_number = frame_number
        det.tracking_plane_loc_x = world_xc
        det.tracking_plane_loc_y = world_yc
        det.image_loc_x = xc
        det.image_loc_y = yc
        det.image_bbox_TL_x = bbox[0]
        det.image_bbox_TL_y = bbox[1]
        det.image_bbox_BR_x = bbox[2]
        det.image_bbox_BR_y = bbox[3]
        det.area = area
        det.lon = lon_lat[0]
        det.lat = lon_lat[1]
        det.alt = 0
        det.timestamp = timestamp

        out_dets.append(det)

    return out_dets


# Generate false alarm detections to a new new track_set
# false detections per km^2 per minute
def pfa_track_set(start_id, time_bounds, frame_bounds, base_layer, false_alarm_rate):
    pfa_tracks = {}
    for i, fa_det in enumerate(generate_false_alarms(time_bounds,
                                                     frame_bounds,
                                                     base_layer,
                                                     false_alarm_rate)):
        fa_track_id = start_id + i + 1
        fa_track = Track()
        fa_track.id = fa_track_id
        fa_track.detections = [fa_det]
        fa_track.compute_bounds()

        pfa_tracks[fa_track_id] = fa_track

    return pfa_tracks


# Modifies the track_set in place and returns it as a convenience
# false detections per km^2 per minute
def add_false_alarms(track_set, base_layer, false_alarm_rate):
    pfa_tracks = pfa_track_set(max(track_set.keys()),
                               TrackSet.time_bounds(track_set),
                               TrackSet.frame_bounds(track_set),
                               base_layer, false_alarm_rate)
    track_set.update(pfa_tracks)
    return track_set

# Turns all detections into single detection tracks
# Creates a single detection from multiple detections in the same pixel on the same frame
# Removes any detections under a specified speed
def clean_track_set(track_set):

    # Copy and remove tracks based on speed
    moving_tracks = {}
    for tid, track in track_set.items():
        new_trk = Track()
        new_trk.id = tid
        moving_tracks[tid] = new_trk
        if len(track.detections)==0:
            print('wtf')
            continue
        last_det = track.detections[0].copy()
        new_trk.detections.append(last_det)
        if len(track.detections) > 1:
            for det in track.detections[1:]:
                # Check to see if the speed between this detection
                # and the last detection is below our threshold
                # If so, don't add it
                avg_speed = Detection.dist(det,last_det)/(det.timestamp - last_det.timestamp)
                if avg_speed > 0.1:  # m/s
                    new_trk.detections.append(det)
                    last_det = det.copy()

    # Sort moving detections by frame
    frames = {}
    for tid, track in moving_tracks.items():
        for det in track.detections:
            if det.frame_number not in frames:
                frames[det.frame_number] = []
            frame_dets = frames[det.frame_number]
            if len(frame_dets) == 0: # Always add a detection if frame is empty
                frame_dets.append(det)
                continue
            # Look for detections already in this frame
            # and check if this new detection pixel location
            # is the same as any detection already in this frame
            px_match = False
            for d in frame_dets:
                if int(d.image_loc_x) == int(det.image_loc_x) and \
                   int(d.image_loc_y) == int(det.image_loc_y):
                    # Same pixel, let's combine them
                    # TODO AVERAGE PIXEL LOCATIONS
                    d.average(det)
                    px_match = True
                    break
            if not px_match:
                frame_dets.append(det)

    # Put each detection on its own track
    new_tid = 0
    det_set = {}
    for fid,frame in frames.items():
        for det in frame:
            trk = Track()
            trk.id = new_tid
            trk.detections.append(det)
            trk.compute_bounds()
            det_set[trk.id] = trk
            new_tid+=1

    # Ok, all clean and ready to go
    return det_set









# Drops detections if they're occluded with respect to an occlusion
# mask.  Modifies the track_set in place and returns it as a
# convenience
def occlude_track_set(track_set, occlusion_mask_layer):
    # Occlusions mask should be of type BaseLayer
    # Actual boolean mask indicating that the pixel is not occluded
    # (blue value of 0 indicates no building occlusion)
    mask = occlusion_mask_layer.image[:, :, 2] == 0

    for tid, track in track_set.items():
        kept_detections = []
        for detection in track.detections:
            im_pt = occlusion_mask_layer.image_coords_from_lon_lat(
                detection.lon,
                detection.lat)
            im_pt = np.round(im_pt).astype(np.int)
            clipped_im_pt = im_pt.clip([0, 0], np.array(mask.T.shape) - 1)
            if not (im_pt == clipped_im_pt).all():
                print(f"Warning: clipping coordinates to mask bounds ({im_pt} "
                      f"to {clipped_im_pt})")
            if mask.T[tuple(clipped_im_pt)]:
                # The pixel is not occluded
                kept_detections.append(detection)

        track.detections = kept_detections
        track.compute_bounds()

    return track_set


# This algorithm randomly drops detections as they pass through an intersection.
# Modifies the track_set in place and returns it as a convenience
def pdintersections_track_set(track_set,
                              base_layer,
                              road_network,
                              probability_of_detection=1.0,
                              radius_m=10):
    intersections = [base_layer.meters_from_lon_lat(*xy) for xy in
                     road_network.node_position(
                         road_network._intersection_nodes)]
    tree = cKDTree(intersections)

    for tid, track in track_set.items():
        pd_intersection_rolls = {}
        drop = False
        pd_detections = []
        for det in track.detections:
            drop = False
            for intersection_i in tree.query_ball_point(
                    base_layer.meters_from_lon_lat(det.lon, det.lat),
                    radius_m):
                if intersection_i not in pd_intersection_rolls:
                    pd_intersection_rolls[intersection_i] = np.random.rand()

                if(pd_intersection_rolls[intersection_i] >
                   probability_of_detection):
                    drop = True

            if not drop:
                pd_detections.append(det)

        track.detections = pd_detections
        track.compute_bounds()

    return track_set


# Computes detection image coordinates via the provided base layer
# using the detections' lon and lat.  Then converts these resampled
# images coordinates back out to world coordinates.
# Modifies the track_set in place and returns it as a convenience
def compute_gsd_world_coordinates(track_set, base_layer):
    # Compute the image coordinates based on the world coordinates
    px.world_to_image_coordinates(base_layer, track_set)
    # Now compute the world coordinates based on those image coordinates
    # Depending on the image GSD, the world coordinates can significantly degrade
    px.image_to_world_coordinates(base_layer, track_set)

    return track_set


def main():
    # Find all kw18 files from a directory wild card
    for root, dirs, files in os.walk("C:/Programming/Trailblazer/data/pNeuma/"):
        for name in files:
            if name.endswith('.csv'):
                csv_file = os.path.join(root, name)
                track_set = read_pNeuma(csv_file)
                TrackSet.compute_bounds(track_set)
                lons,lats = TrackSet.lat_lon_bounds(track_set)
                print("csv : "+csv_file)
                print("Lat Extents:"+str(lats))
                print("Lon Extents:"+str(lons))
                print("\n")
            if name.endswith('.kw18'):
                kw18_file = os.path.join(root, name)
                track_set = read_kw18(kw18_file)
                TrackSet.compute_bounds(track_set)
                lons,lats = TrackSet.lat_lon_bounds(track_set)
                print("kw18 : "+kw18_file)
                print("Lat Extents:"+str(lats))
                print("Lon Extents:"+str(lons))
                print("\n")



                if root.find('convoy')>-1:
                    kw18_file = os.path.join(root, name)
                    convoy_ids_file = os.path.join(root,"sumo_convoy_ids_.txt")
                    if os.path.exists(convoy_ids_file):
                        with open(convoy_ids_file) as id_file:
                            ids = id_file.readline().split(' ')
                        track_set = read_kw18(kw18_file)
                        event_set = {}
                        event = Event()
                        event.id = 0
                        event.type = 7
                        event_set[0] = event
                        for id in ids:
                            track = track_set[int(id)]
                            if track is not None:
                                event.tracks.append(track)
                        kwe_file = root+'/sumo_convoy.kwe'
                        write_kwe(event_set, kwe_file)
                        print("Writing "+kwe_file)



    #track_set = kw_utils.read_kw18(kw18_file)
if __name__ == "__main__":
  main()
