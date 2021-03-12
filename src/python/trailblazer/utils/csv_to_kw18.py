import logging
import argparse
from pyproj import Geod
from trailblazer.traffic.vital import Detection

""" KW18 FORMAT
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

class csv2kw18(object):
    __slots__ = ['kw18_header',
                 'track_ids',
                 'tracks',
                 'wgs84_geod',
                 'log']

    def __init__(self):
        self.log = logging.getLogger("csv2kw18")
        self.tracks = {}
        self.track_ids = {}
        self.wgs84_geod = Geod(ellps='WGS84')
        self.kw18_header = '# 1:Track-id  2:Track-length  3:Frame-number' + \
                           '  4-5:Tracking-plane-loc(x,y)  6-7:velocity(x,y)' + \
                           '  8-9:Image-loc(x,y)' + \
                           '  10-13:Img-bbox(TL_x,TL_y,BR_x,BR_y)' + \
                           '  14:Area  15-17:World-loc(x,y,z) 18:timestamp' + \
                           '  19:object-type-id  20:activity-type-id\n'

    def distance(self, lon1, lat1, lon2, lat2):
        az12, az21, dist = self.wgs84_geod.inv(lon1, lat1, lon2, lat2)
        return dist  # in meters

    def convert(self, csv_file, kw18_file):
        self.log.info("Converting "+csv_file+" to "+kw18_file)
        # Read csv and look for headers as the first line
        with open(csv_file) as csv:
            line_count = 0
            next_track_id = 1
            for line in csv:
                columns = line.split(',')
                # Make zero the first item in the list
                # Use this first item if you don't have data for this kw18 column
                columns.insert(0, 0)

                if line_count == 0:
                    # Is the first line data (i.e. does csv have a header?)
                    if isinstance(columns[1], str):
                        line_count += 1
                        continue  # Skip

                track_uuid = columns[1]  # Fill in which column this is
                # Convert the uuid to an int id
                if track_uuid not in self.track_ids:
                    self.track_ids[track_uuid] = next_track_id
                    next_track_id += 1
                track_id = self.track_ids[track_uuid]

                # Which track are we on
                if track_id not in self.tracks:
                    self.tracks[track_id] = []
                tracks = self.tracks[track_id]

                # Based on what your csv is,
                # change the column index's to the 1 based index of the csv column for a specific kw18 value
                # (because columns[0] is reserved to always be 0, and not provided in the csv)
                detection = Detection()
                detection.frame_number = columns[0]
                detection.tracking_plane_loc_x = columns[0]
                detection.tracking_plane_loc_y = columns[0]
                detection.velocity_x = columns[0]
                detection.velocity_y = columns[0]
                detection.image_loc_x = columns[0]
                detection.image_loc_y = columns[0]
                detection.image_bbox_TL_x = columns[0]
                detection.image_bbox_TL_y = columns[0]
                detection.image_bbox_BR_x = columns[0]
                detection.image_bbox_BR_y = columns[0]
                detection.area = columns[0]
                detection.world_loc_x = columns[0]
                detection.world_loc_y = columns[0]
                detection.world_loc_z = columns[0]
                detection.timestamp = columns[0]
                tracks.append(detection)
                line_count+=1

        with open(kw18_file, 'w') as kw18:
            kw18.write(self.kw18_header)
            for tid,detections in self.tracks.items():
                for det in detections:
                    kw18.write(str(tid) + ' ' +                      # 1:Track-id
                               str(len(detections)) + ' ' +          # 2:Track-length
                               str(det.frame_number) + ' ' +         # 3:Frame-number
                               str(det.tracking_plane_loc_x) + ' ' + # 4:Tracking-plane-loc(x)
                               str(det.tracking_plane_loc_x) + ' ' + # 5:Tracking-plane-loc(y)
                               str(det.velocity_x) + ' ' +           # 6:velocity(x)
                               str(det.velocity_y) + ' ' +           # 7:velocity(y)
                               str(det.image_loc_x) + ' ' +          # 8:Image-loc(x)
                               str(det.image_loc_y) + ' ' +          # 9:Image-loc(y)
                               str(det.image_bbox_TL_x) + ' ' +      # 10:Img-bbox(TL_x)
                               str(det.image_bbox_TL_y) + ' ' +      # 11:Img-bbox(TL_y)
                               str(det.image_bbox_BR_x) + ' ' +      # 12:Img-bbox(BR_x)
                               str(det.image_bbox_BR_y) + ' ' +      # 13:Img-bbox(BR_y)
                               str(det.area) + ' ' +                 # 14:Area
                               str(det.world_loc_x) + ' ' +          # 15:World-loc(x/lon)
                               str(det.world_loc_y) + ' ' +          # 16:World-loc(y/lat)
                               str(det.world_loc_z) + ' ' +          # 17:World-loc(z/alt)
                               str(det.timestamp) + '\n'             # 18:timestamp
                               )

def main():
    logger.initialize()
    # Setup input arguments
    parser = argparse.ArgumentParser(description='Convert a csv file to a new kw18 file');
    parser.add_argument("csv", type=str)
    parser.add_argument("kw18", type=str)
    args = parser.parse_args()
    me = csv2kw18()
    me.convert(args.csv, args.kw18)

if __name__ == '__main__':
    main()