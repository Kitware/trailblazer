#! /usr/bin/python

import os
import argparse
import sumolib
import xml.etree.cElementTree as ET
from math import cos, sin, radians

from trailblazer.sumo.add_traffic import RouteBuilder
from trailblazer.utils.path_utils import fixup_paths
from trailblazer.traffic.vital import Detection, Track, TrackSet, Event

class FCD(object):

    # Splitting velocity into x and y components
    @staticmethod
    def calc_velocity(speed, angle):
        rad = radians(90 - angle)
        vel_x = speed * cos(rad)
        vel_y = speed * sin(rad)
        return vel_x, vel_y

    @staticmethod
    def get_vehicles(fcd_file, search):
        if not fcd_file.endswith('.xml'):
            fcd_file = fcd_file + '/osm.fcd.xml'
        frame_tree = ET.parse(fcd_file)
        vehicle_data = frame_tree.getroot()

        search_vehicles = {}
        for timestep in vehicle_data:
            time = float(timestep.attrib['time'])
            for vehicle in timestep:
                type = vehicle.attrib['type']
                if search in type:
                    if time not in search_vehicles:
                        search_vehicles[time] = []
                    ary = search_vehicles[time]
                    ary.append( {'id':vehicle.attrib['id'],
                                 'x':float(vehicle.attrib['x']),
                                 'y':float(vehicle.attrib['y']),
                                 'speed':float(vehicle.attrib['speed']),
                                 'lane':vehicle.attrib['lane'] } )
        return search_vehicles

    @staticmethod
    def convert2kw18(sumo_net, fcd_file, start_time, stop_time, search=None):
        print("Coverting fcd to tracks...")
        track_set = {}
        event_set = {}
        if search is not None:
            # We only search for 1 thing, so 1 event
            # Currently the event type (7) is for convoy, which is 'following' in the PerSEAS type set
            event = Event()
            event.id = 0
            event.type = 7
            event_set[event.id] = event

        vehicle_ids = {}
        next_track_id = 0
        if not fcd_file.endswith('.xml'):
            fcd_file = fcd_file + '/osm.fcd.xml'

        print("Parsing xml document...")
        frame_tree = ET.parse(fcd_file)
        vehicle_data = frame_tree.getroot()

        print("Sorting Detections..")
        frame_number = 0
        print("Translating frames...")
        for timestep in vehicle_data:
            time = float(timestep.attrib['time'])
            if time < start_time:
                continue
            if time > stop_time:
                break
            for vehicle in timestep:
                detection = Detection()
                vehicle_uuid = vehicle.attrib['id']
                # Convert the uuid to an int id
                if vehicle_uuid not in vehicle_ids:
                    vehicle_ids[vehicle_uuid] = next_track_id
                    next_track_id += 1
                track_id = vehicle_ids[vehicle_uuid]
                # Get the track associated with this vehicle time stamp
                if track_id not in track_set.keys():
                    track_set[track_id] = Track()
                track = track_set[track_id]
                track.id = track_id
                if search is not None and search in vehicle_uuid and not event.contains(track_id):
                    event.tracks.append(track)
                # Create a new detection for this entry
                detection = Detection()
                track.detections.append(detection)
                # Check for type encoding such as the one in add_convoy
                # All type encodings should follow a pattern of
                #   - <name>_<type>_<id>
                #   - <name>_<type>
                # where type is a SUMO type we have in vehicle_specs
                vehicle_type = vehicle.attrib['type']
                type_list = vehicle_type.split('_')
                if len(type_list) >= 2:
                    vehicle_type = type_list[1]
                vehicle_length = 0.0
                vehicle_width = 0.0
                for spec in RouteBuilder.vehicle_specs:
                    if spec['type'] == vehicle_type:
                        vehicle_length = spec['length']
                        vehicle_width = spec['width']
                        break
                if vehicle_length == 0:
                    print('Could not find vehicle spec for type ' + vehicle_type)
                    continue
                detection.timestamp = time - start_time
                detection.frame_number = frame_number
                # This will be the X,Y values of the UTM position
                detection.tracking_plane_loc_x = float(vehicle.attrib['x'])
                detection.tracking_plane_loc_y = float(vehicle.attrib['y'])
                angle = float(vehicle.attrib['angle'])
                speed = float(vehicle.attrib['speed'])
                detection.velocity_x, detection.velocity_y = FCD.calc_velocity(speed, angle)
                # No image associated with fcd
                detection.image_loc_x = 0
                detection.image_loc_y = 0
                # This should be in image coordinates, but since there are none
                # I am just going to put in the vehicle size
                detection.image_bbox_TL_x = 0
                detection.image_bbox_TL_y = vehicle_width
                detection.image_bbox_BR_x = vehicle_length
                detection.image_bbox_BR_y = 0
                # I could offset from the vehicle UTM coords...
                #detection.image_bbox_TL_x = detection.tracking_plane_loc_x + (vehicle_length * 0.5)
                #detection.image_bbox_TL_y = detection.tracking_plane_loc_y + (vehicle_width * 0.5)
                #detection.image_bbox_BR_x = detection.tracking_plane_loc_x - (vehicle_length * 0.5)
                #detection.image_bbox_BR_y = detection.tracking_plane_loc_y - (vehicle_width * 0.5)
                # Area will also be the area of the vehicle box
                detection.area = vehicle_width * vehicle_length
                # Have sumo compute the lat/lon based on the UTM coordinates
                lon, lat = sumo_net.convertXY2LonLat(detection.tracking_plane_loc_x, detection.tracking_plane_loc_y)
                detection.lon = lon
                detection.lat = lat
                detection.alt = 0
            frame_number += 1
        vehicle_data = None
        frame_tree = None
        os.remove(fcd_file)

        TrackSet.compute_bounds(track_set)
        return track_set, event_set

def main():
    parser = argparse.ArgumentParser(description='Start sumo with a network populated with a directory of route files.')
    parser.add_argument('--net_directory',
                        help='Directory path containing .net.xml file.',
                        type=str,
                        default='./net_quantico/')
    parser.add_argument('--fcd_file',
                        help='Input fcd file.',
                        type=str,
                        default="N:/Programming/BIGT4/sumo_tools/net_quantico/routes/traffic0/low_density/run0/osm.fcd.xml")
    parser.add_argument('--kw18_file',
                        help='kw18 file to write out.',
                        type=str,
                        default="N:/Programming/BIGT4/sumo_tools/net_quantico/routes/traffic0/low_density/run0/sumo.kw18")
    parser.add_argument('--start',
                        help='When to start converting data from the simulation.',
                        type=float,
                        default=200)
    parser.add_argument('--stop',
                        help='When to stop converting data from the simulation.',
                        type=float,
                        default=600)

    args = parser.parse_args()
    network_dir, fcd_file = fixup_paths(args.net_directory, args.fcd_file)
    network_dir, kw18_file = fixup_paths(args.net_directory, args.kw18_file)

    net_xml = args.net_directory + "osm.net.xml"
    sumo_net = sumolib.net.readNet(net_xml)
    if not sumo_net:
        print('Could not open network ' + net_xml)

    last_time = FCD.last_time_in_scene(fcd_file, 'convoy')
    print('Found the last convoy at time ' + str(last_time))
    if last_time == -1:
        last_time = float(args.stop)
    track_set,search_set = FCD.convert2kw18(sumo_net, fcd_file, kw18_file, args.start, last_time, 'convoy')



if __name__ == '__main__':
    main()
