# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import numpy as np
from math import floor, ceil
from trailblazer.utils import coordinate_converter as cc

# Pixels for a car (1m / pixel)
# We could put the car size in the kw18 from sumo
#_bbox_x_offset = 1.7 * 0.5
#_bbox_y_offset = 4.7 * 0.5
# Or keep it square, 2m by 2m...
# We don't get orientation...
_bbox_x_offset = 2
_bbox_y_offset = 2

def world_to_image_coordinates(base_layer, track_set):
    gsd = base_layer.gsd[0]
    for tid, track in track_set.items():
        for d in track.detections:
            # Update the tracking plane coordinates with enu from the base layer origin
            enu = cc.llh_to_enu(d.lat,d.lon,0,base_layer.lat0,base_layer.lon0,0)
            d.tracking_plane_loc_x = enu[0]
            d.tracking_plane_loc_y = enu[1]
            image_coords = base_layer.image_coords_from_lon_lat(d.lon, d.lat)
            # Update the image and bounding box pixels
            d.image_loc_x = image_coords[0]
            d.image_loc_y = image_coords[1]
            d.image_bbox_TL_x = image_coords[0] - (_bbox_x_offset / gsd)
            d.image_bbox_TL_y = image_coords[1] + (_bbox_y_offset / gsd)
            d.image_bbox_BR_x = image_coords[0] + (_bbox_x_offset / gsd)
            d.image_bbox_BR_y = image_coords[1] - (_bbox_y_offset / gsd)

def image_to_world_coordinates(base_layer, track_set):
    for tid, track in track_set.items():
        for det in track.detections:
            # Compute world coordinates from image coordinates
            det.lon, det.lat = \
                base_layer.lon_lat_from_image_coords([det.image_loc_x, det.image_loc_y])
            # Compute the tracking coordinates from image coordinates
            det.tracking_plane_loc_x, det.tracking_plane_loc_y = \
                base_layer.meters_from_image_coords([det.image_loc_x, det.image_loc_y])
        # Update the track bounds
        track.compute_bounds()

def track_pixels(track, blur_radius=0, plot_results=False):
    """Return the integer pixel coordinates and associated times.

    The track trajectory in the image coordinate system may have floating-
    point values due to smoothing operations or from being tranformed from
    a higher-resolution image to the lower-resolution one currently
    considered. Pixels coordinates are integer valued at the center of the
    pixels with pixel cell boundaries occurring at +/- 0.5. This method
    returns the continuous progression of pixel column and row coordinates
    for each pixel cell entered by the track trajectory. Pixels are counted
    once per time that the trajectory enters. Therefore, if the trajectory
    revisits a pixel later in the trajectory, it will be listed again on
    the subsequent re-entry.

    Associated with each listing of a pixel is the time corresponding to
    the closest approach (approximately) to the cell center. This can be
    used to query other trajectory properties (e.g., velocity) associated
    with each pixel intersection.

    Assumes a linear interpolation of raw_image_coords.

    :return: List of pixel (row,column) indices and an associated list of
        the time that the trajectory was nearest to the center of that
        pixel.
    :rtype: list of 2 lists

    """
    if blur_radius != 0:
        raise NotImplementedError('Blurring has not been implemented.')

    t = []
    x = []
    y = []
    for d in track.detections:
        t.append(d.timestamp)
        x.append(d.image_loc_x)
        y.append(d.image_loc_y)

    pixels, pixel_times = unique_pixels(t, x, y)

    if False:
        # Duplicate curve multiple times.
        pixels = []
        pixel_times = []
        for i in range(-1 ,2):
            for j in range(-1 ,2):
                ret = unique_pixels(t, np.array(x) +i, np.array(y) +j)
                pixels = pixels + ret[0]
                pixel_times = pixel_times + ret[1]

    pixels = np.array(pixels, dtype=np.int)
    pixel_times = np.array(pixel_times)


    return pixels, pixel_times

def unique_pixels(t, x, y):
    """Return list of unique pixel indices visited by the trajectory.

    Given a floating-point array of image pixel coordinates, this function
    returns a list of pixel (column, row) coordinates for each pixel entered by
    the trajectory. The trajectory is assumed to be piecewise linear. Pixels
    coordinates are integer valued at the center of the pixels with pixel cell
    boundaries occurring at +/- 0.5. Pixels are counted once per time that the
    trajectory enters the pixel cell. Therefore, if the trajectory revisits a
    pixel later in the trajectory, it will be listed again for the subsequent
    re-entry.

    Associated with each listing of a pixel is the time corresponding to
    the closest approach (approximately) to the cell center. This can be
    used to query other trajectory properties (e.g., velocity) associated
    with each pixel intersection.

    Ideally, we'd like the times where the curve comes closest to the center
    of the pixel. However, a decent approximation is to take the average of the
    times and locations where the curve enters and then exits the pixel.
    Handling at the two endpoints of the full trajectory average between the
    endpoint and the one intersection point. Therefore, this can be a worse
    approximation at the ends.

    :param t: Times associated with the 'x' and 'y' lists.
    :type t:

    :param x: Horizontal image coordinate (i.e., image column indices).
    :type x: list

    :param y: Vertical image coordinate (i.e., image row indices).
    :type y: list

    :return: List of pixel (row, column) indices and an associated list of the
        time that the trajectory was nearest to the center of that pixel.
    :rtype: list of 2 lists

    """
    T = [t[0]]
    X = [x[0]]
    Y = [y[0]]
    for i in range(len(t)-1):
        t1 = t[i]
        t2 = t[i+1]
        x1 = x[i]
        y1 = y[i]
        x2 = x[i+1]
        y2 = y[i+1]
        dt = t2 - t1
        dxdt = (x2-x1)/(t2-t1)
        dydt = (y2-y1)/(t2-t1)

        while True:
            if dxdt > 0:
                next_t_x = (floor(x1 + 0.5) + 0.5 - x1)/dxdt
            elif dxdt < 0:
                next_t_x = (ceil(x1 - 0.5) - 0.5 - x1)/dxdt
            else:
                next_t_x = float('inf')

            if dydt > 0:
                next_t_y = (floor(y1 + 0.5) + 0.5 - y1)/dydt
            elif dydt < 0:
                next_t_y = (ceil(y1 -0.5) - 0.5 - y1)/dydt
            else:
                next_t_y = float('inf')

            next_t = min(next_t_x, next_t_y)

            if next_t > dt:
                # This line segment never makes it out of the cell.
                break
            else:
                t1 = t1 + next_t
                x1 = x1 + next_t*dxdt
                y1 = y1 + next_t*dydt
                T.append(t1)
                X.append(x1)
                Y.append(y1)
                dt = t2 - t1

    T.append(t[-1])
    X.append(x[-1])
    Y.append(y[-1])

    pixels = [[int(floor((Y[i]+Y[i+1])/2+0.5)),
               int(floor((X[i]+X[i+1])/2+0.5))]
              for i in range(len(X)-1)]
    pixel_times = [(T[i] + T[i+1])/2 for i in range(len(T) - 1)]
    return pixels, pixel_times