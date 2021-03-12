import sys
import numpy as np
from pyproj import Geod
from shapely.geometry import Point

_wgs84_geod = Geod(ellps='WGS84')

class Detection(object):
    __slots__ = ['frame_number',
                 'tracking_plane_loc_x',
                 'tracking_plane_loc_y',
                 'velocity_x',
                 'velocity_y',
                 'image_loc_x',
                 'image_loc_y',
                 'image_bbox_TL_x',
                 'image_bbox_TL_y',
                 'image_bbox_BR_x',
                 'image_bbox_BR_y',
                 'area',
                 'lon',
                 'lat',
                 'alt',
                 'timestamp']

    def __init__(self):
        self.frame_number = 0
        self.tracking_plane_loc_x = 0
        self.tracking_plane_loc_y = 0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.image_loc_x = 0
        self.image_loc_y = 0
        self.image_bbox_TL_x = 0
        self.image_bbox_TL_y = 0
        self.image_bbox_BR_x = 0
        self.image_bbox_BR_y = 0
        self.area = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.timestamp = 0.0

    def copy(self):
        to = Detection()
        to.frame_number = self.frame_number
        to.tracking_plane_loc_x = self.tracking_plane_loc_x
        to.tracking_plane_loc_y = self.tracking_plane_loc_y
        to.velocity_x = self.velocity_x
        to.velocity_y = self.velocity_y
        to.image_loc_x = self.image_loc_x
        to.image_loc_y = self.image_loc_y
        to.image_bbox_TL_x = self.image_bbox_TL_x
        to.image_bbox_TL_y = self.image_bbox_TL_y
        to.image_bbox_BR_x = self.image_bbox_BR_x
        to.image_bbox_BR_y = self.image_bbox_BR_y
        to.area = self.area
        to.lat = self.lat
        to.lon = self.lon
        to.alt = self.alt
        to.timestamp = self.timestamp
        return to

    def average(self, d):
        pass

    @staticmethod
    def dist(d1, d2):
        az12, az21, dist = _wgs84_geod.inv(d1.lon, d1.lat, d2.lon, d2.lat)
        return dist  # in meters


class Track(object):
    __slots__ = ['id',
                 'start_time',
                 'start_frame',
                 'end_time',
                 'end_frame',
                 'min_x',
                 'min_y',
                 'max_x',
                 'max_y',
                 'min_lat',
                 'min_lon',
                 'max_lat',
                 'max_lon',
                 'detections']

    def __init__(self):
        self.id = -1
        self.detections = []
        self.clear_bounds()

    def clear_bounds(self):
        self.start_time = sys.maxsize
        self.start_frame = sys.maxsize
        self.end_time = 0
        self.end_frame = 0
        self.min_x = sys.maxsize
        self.min_y = sys.maxsize
        self.max_x = 0
        self.max_y = 0
        self.min_lat = 90
        self.min_lon = 180
        self.max_lat = -90
        self.max_lon = -180

    def compute_bounds(self):
        self.clear_bounds()
        for detection in self.detections:
            self.start_time = min(self.start_time, detection.timestamp)
            self.start_frame = min(self.start_frame, detection.frame_number)
            self.end_time = max(self.end_time, detection.timestamp)
            self.end_frame = max(self.end_frame, detection.frame_number)
            self.min_x = min(self.min_x, detection.image_bbox_TL_x)
            self.min_y = min(self.min_y, detection.image_bbox_BR_y)
            self.max_x = max(self.max_x, detection.image_bbox_BR_x)
            self.max_y = max(self.max_y, detection.image_bbox_TL_y)
            self.min_lat = min(self.min_lat, detection.lat)
            self.min_lon = min(self.min_lon, detection.lon)
            self.max_lat = max(self.max_lat, detection.lat)
            self.max_lon = max(self.max_lon, detection.lon)


class TrackSet(object):

    @staticmethod
    def compute_bounds(track_set):
        for tid, track in track_set.items():
            track.compute_bounds()

    @staticmethod
    def lat_lon_bounds(track_set):
        """Return minimum and maximum for latitude and longitude (degrees)."""
        lat = [90, -90]
        lon = [180, -180]
        for tid,track in track_set.items():
            lat[0] = min(track.min_lat, lat[0])
            lat[1] = max(track.max_lat, lat[1])
            lon[0] = min(track.min_lon, lon[0])
            lon[1] = max(track.max_lon, lon[1])

        return lat, lon

    @staticmethod
    def frame_bounds(track_set):
        start_frame, end_frame = np.inf, -np.inf
        for track in track_set.values():
            start_frame = min(track.start_frame, start_frame)
            end_frame = max(track.end_frame, end_frame)

        return start_frame, end_frame

    @staticmethod
    def time_bounds(track_set):
        start_time, end_time = np.inf, -np.inf
        for track in track_set.values():
            start_time = min(track.start_time, start_time)
            end_time = max(track.end_time, end_time)

        return start_time, end_time

    @staticmethod
    def apply_mask(track_set, polygon, start_time, end_time):
        new_track_set = {}
        for track in track_set.values():
            new_track = Track()
            new_track.id = track.id
            for d in track.detections:
                point = (d.lon, d.lat)
                if start_time <= d.timestamp <= end_time and polygon.contains(Point(*point)):
                    continue
                new_d = d.copy()
                new_track.detections.append(new_d)
            if len(new_track.detections) > 0:
                new_track.compute_bounds()
                new_track_set[new_track.id] = new_track
        return new_track_set

    @staticmethod
    def apply_xmask(track_set, polygon):
        new_track_set = {}
        for track in track_set.values():
            new_track = Track()
            new_track.id = track.id
            for d in track.detections:
                point = (d.lon, d.lat)
                if not polygon.contains(Point(*point)):
                    continue
                new_d = d.copy()
                new_track.detections.append(new_d)
            if len(new_track.detections) > 0:
                new_track.compute_bounds()
                new_track_set[new_track.id] = new_track
        return new_track_set


class Event(object):
    __slots__ = ['id',
                 'type',
                 'start_time',
                 'start_frame',
                 'end_time',
                 'end_time',
                 'end_frame',
                 'min_x',
                 'min_y',
                 'max_x',
                 'max_y',
                 'tracks']

    def __init__(self):
        self.start_time = sys.maxsize
        self.start_frame = sys.maxsize
        self.end_time = 0
        self.end_frame = 0
        self.min_x = sys.maxsize
        self.min_y = sys.maxsize
        self.max_x = 0
        self.max_y = 0
        self.tracks = []

    def contains(self, track_id):
        for track in self.tracks:
            if track.id == track_id:
                return True
        return False

    def compute_bounds(self):
        self.start_time = sys.maxsize
        self.start_frame = sys.maxsize
        self.end_time = 0
        self.end_frame = 0
        self.min_x = sys.maxsize
        self.min_y = sys.maxsize
        self.max_x = 0
        self.max_y = 0
        for track in self.tracks:
            track.compute_bounds()
            self.start_time = min(self.start_time, track.start_time)
            self.start_frame = min(self.start_frame, track.start_frame)
            self.end_time = max(self.end_time, track.end_time)
            self.end_frame = max(self.end_frame, track.end_frame)
            self.min_x = min(self.min_x, track.min_x)
            self.min_y = min(self.min_y, track.min_y)
            self.max_x = max(self.max_x, track.max_x)
            self.max_y = max(self.max_y, track.max_y)
