import cv2
import numpy as np
from osgeo import osr, gdal
from trailblazer.traffic.vital import TrackSet
from trailblazer.utils import coordinate_converter as cc, pixel_utils as px


class BaseLayer(object):
    def __init__(self):
        self._ds = None
        self._image = None
        self._image_cs = None
        self._lon0 = None
        self._lat0 = None
        self._gsd = None

        # Create lat/lon coordinate system.
        wgs84_wkt = """
        GEOGCS["WGS 84",
            DATUM["WGS_1984",
                SPHEROID["WGS 84",6378137,298.257223563,
                    AUTHORITY["EPSG","7030"]],
                AUTHORITY["EPSG","6326"]],
            PRIMEM["Greenwich",0,
                AUTHORITY["EPSG","8901"]],
            UNIT["degree",0.01745329251994328,
                AUTHORITY["EPSG","9122"]],
            AUTHORITY["EPSG","4326"]]"""
        self._wgs84_cs = osr.SpatialReference()
        self._wgs84_cs.ImportFromWkt(wgs84_wkt)

    @staticmethod
    def open(fname):
        """Open a base layer image.
        """
        base_layer = BaseLayer()

        ds = gdal.Open(fname, gdal.GA_ReadOnly)

        if ds is None:
            raise OSError()

        base_layer.ds = ds

        return base_layer

    @staticmethod
    def open_with_corner_bounds(fname, corner1_lon, corner1_lat,
                                corner2_lon, corner2_lat, corner3_lon,
                                corner3_lat, corner4_lon, corner4_lat):
        """Open an image and apply lon-lat bounds.

        If the image is a standard image format without geographic metadata,
        the latitude and longitude of the four corners of the image can be
        specified. All eight parameters must be specified in this case.

        corner1 - upper-left corner
        corner2 - upper-right corner
        corner3 - lower-right corner
        corner4 - lower-left corner

        :param corner1_lon: Longitude (degrees) for first corner.
        :type corner1_lon: float

        :param corner1_lat: Latitude (degrees) for first corner.
        :type corner1_lat: float

        :param corner2_lon: Longitude (degrees) for second corner.
        :type corner2_lon: floatread_kw18

        :param corner2_lat: Latitude (degrees) for second corner.
        :type corner2_lat: float

        :param corner3_lon: Longitude (degrees) for third corner.
        :type corner3_lon: float

        :param corner3_lat: Latitude (degrees) for third corner.
        :type corner3_lat: float

        :param corner4_lon: Longitude (degrees) for fourth corner.
        :type corner4_lon: float

        :param corner4_lat: Latitude (degrees) for fourth corner.
        :type corner4_lat: float

        """
        self = BaseLayer()

        ds = gdal.Open(fname, gdal.GA_ReadOnly)

        if ds is None:
            raise Exception()

        ll_bbox = [corner1_lon,corner1_lat,corner2_lon,corner2_lat,corner3_lon,
                   corner3_lat,corner4_lon,corner4_lat]

        # Check that ll_bbox should either be all None or all not None.
        assert np.all([l is not None for l in ll_bbox]), 'Must specify ' \
            'lat and lone for the upper left and bottom right of the '   \
            'image.'
        ds.SetProjection(self._wgs84_cs.ExportToWkt())

        # Define warping from pixel/line (P,L) raster space to (lon,lat).
        # Xp = padfTransform[0] + P*padfTransform[1] + L*padfTransform[2];
        # Yp = padfTransform[3] + P*padfTransform[4] + L*padfTransform[5];

        ll_bbox = np.reshape(np.array(ll_bbox, dtype=np.float64), (4,2))
        res_x = ds.RasterXSize
        res_y = ds.RasterYSize
        rc_bbox = np.array([[0,0],[res_x,0],[res_x,res_y],[0,res_y]],
                           dtype=np.float64)

        A = cv2.estimateRigidTransform(np.reshape(rc_bbox, (1,-1,2)),
                                       np.reshape(ll_bbox, (1,-1,2)), True)

        geotrans = [A[0,2],A[0,0],A[0,1],A[1,2],A[1,0],A[1,1]]
        ds.SetGeoTransform(geotrans)

        self.ds = ds

        # Check results.
        errs = []
        for i in range(4):
            err = rc_bbox[i] - self.image_coords_from_lon_lat(ll_bbox[i,0],
                         ll_bbox[i,1])
            errs.append(np.sqrt(np.sum(err**2)))

        print('Fitting errors at the corners are %0.1f, %0.1f, %0.1f, %0.1f '
              'pixels' % tuple(errs))

        return self

    @staticmethod
    def empty_wgs_84_image(lon_range, lat_range, gsd=1.0, bands=3):
        """Create empty image with a WGS84 projection.

        The GDAL projection is set to the WGS84 coordinate system but with a
        GeoTransform defined such that at the center of the image, pixels are
        gsd meters high by gsd meters wide. This accounts for the fact that a
        degree of longitude does not subtent the same arc as a degree of
        latitude, except at the equator. This isotropy breaks down off from the
        center of the image, but is generally still reasonably isotropic for
        domains on the order of kilometers.

        """
        base_layer = BaseLayer()

        min_lon = min(lon_range)
        max_lon = max(lon_range)
        min_lat = min(lat_range)
        max_lat = max(lat_range)
        lat0 = (min_lat + max_lat)/2
        lon0 = (min_lon + max_lon)/2

        # The image coordinates will be proportional to longitude (along image
        # x) and latitude (along image y). Want to set the proportionality
        # constant so that the GSD is equal to 'gsd' along both directions at
        # the center of the image.
        dtheta = 1e-5   # approximately a meter at equator
        dnorth = cc.llh_to_enu(lat0 + dtheta, lon0, 0, lat0, lon0, 0,
                            in_degrees=True)[1]
        deast = cc.llh_to_enu(lat0, lon0 + dtheta, 0, lat0, lon0, 0,
                           in_degrees=True)[0]
        lon_per_pixel = dtheta/deast * gsd
        lat_per_pixel = dtheta/dnorth * gsd

        # Determine image bounds
        res_y = int(abs(np.ceil((max_lat - min_lat)/lat_per_pixel))) + 1
        res_x = int(abs(np.ceil((max_lon - min_lon)/lon_per_pixel))) + 1

        gdal_drv = gdal.GetDriverByName('MEM')
        ds = gdal_drv.Create('', res_x, res_y, bands, gdal.GDT_Byte)
        ds.SetProjection(base_layer._wgs84_cs.ExportToWkt())

        # Define warping from pixel/line (P,L) raster space to (lon,lat).
        # Xp = padfTransform[0] + P*padfTransform[1] + L*padfTransform[2];
        # Yp = padfTransform[3] + P*padfTransform[4] + L*padfTransform[5];
        geotrans = [min_lon, lon_per_pixel, 0, max_lat, 0, -lat_per_pixel]

        ds.SetGeoTransform(geotrans)
        base_layer.ds = ds
        return base_layer

    @staticmethod
    def from_road_network(road_network, gsd=1.0, bkg_color=(0,0,0), road_color=(255,0,0)):
        """Create a rendered BaseLayer image object from the road network.

        :param road_network: Road network.
        :type road_network: RoadNetwork

        """
        G = road_network.network
        [min_lat,max_lat],[min_lon,max_lon] = road_network.lat_lon_bounds()

        base_layer = BaseLayer.empty_wgs_84_image([min_lon,max_lon],
                                                  [min_lat,max_lat], gsd)

        # Draw the image.
        image = np.zeros((base_layer.res_y,base_layer.res_x,3), np.uint8)
        image[:] = bkg_color
        pos = dict(zip(G.node.keys(),
                       [(G.node[x]['longitude'],G.node[x]['latitude'])
                        for x in G.node.keys()]))

        for edge in G.edges():
            x1,y1 = base_layer.image_coords_from_lon_lat(*pos[edge[0]])
            x2,y2 = base_layer.image_coords_from_lon_lat(*pos[edge[1]])

            # The subtracted 0.5 is to account for the difference between image
            # coordinates and pixel indices.
            x1 = int(np.round(x1 - 0.5))
            x2 = int(np.round(x2 - 0.5))
            y1 = int(np.round(y1 - 0.5))
            y2 = int(np.round(y2 - 0.5))

            thickness = int(G.get_edge_data(*edge).get('lanes', 1)) + 1
            cv2.line(image, (x1,y1), (x2,y2), color=road_color,
                     thickness=thickness)

        if False:
            # Test
            image[0,0,:] = (255,255,255)
            image[-1,-1,:] = (255,255,255)
            print('Upper left', base_layer.meters_from_image_coords([0.5,0.5]))
            print('Bottom right',
                  base_layer.meters_from_image_coords([base_layer.res_x-0.5,
                                                       base_layer.res_y-0.5]))

        # Write the Numpy image to the GDAL raster array.
        base_layer.image = image

        return base_layer

    @staticmethod
    def from_track_list(track_set, gsd=1.0, bkg_color=(255,255,255), road_color=(0,0,0)):
        """Create a rendered BaseLayer image object from the road network.

        :param track_set: All tracks.
        :type track_set: a map of Track objects with track id as the key

        :param color: Color of the road segments.
        :type color: None | 3-tuple

        :return: Base layer object and track list with image coordinates
            updated for the newly created base layer.
        :rtype: list of BaseLayer and TrackList

        """
        lat_range,lon_range = TrackSet.lat_lon_bounds(track_set)
        base_layer = BaseLayer.empty_wgs_84_image(lon_range, lat_range, gsd, bands=3)
        # Write new image coordinates to each track detection
        px.lan_lon_to_image_coordinates(base_layer, track_set)

        # Draw the image.
        image = np.zeros((base_layer.res_y,base_layer.res_x,3), np.uint8)
        image[:] = bkg_color

        for tid,track in track_set.items():
            # Color each edge
            if road_color is None:
                while True:
                    c = np.round(np.random.rand(3)*255).astype(np.uint8)
                    if np.linalg.norm(c) > 50:
                        break
            else:
                c = road_color

            for row,col in px.track_pixels(track)[0]:
                image[row,col,:] = c

        # Write the Numpy image to the GDAL raster array.
        base_layer.image = image

        return base_layer

    def save_geotiff(self, fname):
        """Save GeoTIFF image to file.

        """
        # Perform the projection/resampling
        gdal_drv = gdal.GetDriverByName('GTiff')
        dest_ds = gdal_drv.Create(fname, self.res_x, self.res_y,
                                  self.ds.RasterCount, gdal.GDT_Byte)
        dest_ds.SetProjection(self.ds.GetProjection())
        dest_ds.SetGeoTransform(self.ds.GetGeoTransform())

        if self.ds.RasterCount == 1:
            dest_ds.GetRasterBand(1).WriteArray(self.image[:,:], 0, 0)
        else:
            for i in range(self.ds.RasterCount):
                dest_ds.GetRasterBand(i+1).WriteArray(self.image[:,:,i], 0, 0)

        dest_ds.FlushCache()  # Write to disk.

    @staticmethod
    def inter_base_layer_warp_matrix(base_layer_src, base_layer_dst):
        """Return matrix to warp coordinates from base_layer_src to base_layer_dst.

        This funciton returns an affine matrix that warps coordinates from
        the 'base_layer_src' coordinate system to the 'base_layer_dst' coordinate
        system. Both base layers must be defined with the same projection so that
        their difference can be represented by an affine transform.

        :param base_layer_src: Base layer to warp from.
        :type base_layer_src: BaseLayer

        :param base_layer_dst: Base layer to warp to.
        :type base_layer_dst: BaseLayer

        """
        geotrans_src = base_layer_src.ds.GetGeoTransform()
        h_src = np.array([[geotrans_src[1], geotrans_src[2], geotrans_src[0]],
                          [geotrans_src[4], geotrans_src[5], geotrans_src[3]],
                          [0, 0, 1]])

        geotrans_dst = base_layer_dst.ds.GetGeoTransform()
        h_dst = np.array([[geotrans_dst[1], geotrans_dst[2], geotrans_dst[0]],
                          [geotrans_dst[4], geotrans_dst[5], geotrans_dst[3]],
                          [0, 0, 1]])

        return np.dot(np.linalg.inv(h_dst), h_src)

    @property
    def ds(self):
        if self._ds is None:
            raise Exception('You must create or open a model first.')

        return self._ds

    @ds.setter
    def ds(self, val):
        """Reset cached values and recalculate some.

        """
        self._ds = val
        self._image = None
        self._image_cs = None
        self._lon0 = None
        self._lat0 = None
        self._gsd = None

        # Create a transform object to convert between projection of the image
        # and WGS84 coordinates.
        self._to_lla_tform = osr.CoordinateTransformation(self.image_cs,
                                                          self._wgs84_cs)
        self._from_lla_tform = osr.CoordinateTransformation(self._wgs84_cs,
                                                            self.image_cs)

        # See documentation for gdal GetGeoTransform().
        geo_tform_mat = np.identity(3)
        c = self.ds.GetGeoTransform()
        geo_tform_mat[0,0] = c[1]
        geo_tform_mat[0,1] = c[2]
        geo_tform_mat[0,2] = c[0]
        geo_tform_mat[1,0] = c[4]
        geo_tform_mat[1,1] = c[5]
        geo_tform_mat[1,2] = c[3]
        self.geo_inv_tform_mat = np.linalg.inv(geo_tform_mat)[:2]
        self.geo_tform_mat = geo_tform_mat[:2]

        self._lon0,self._lat0 = self.lon_lat_from_image_coords([self.res_x/2,
                                                                self.res_y/2])

    @property
    def lat0(self):
        """Latitude (degrees) at the center of the base layer image.

        """
        return self._lat0

    @property
    def lon0(self):
        """Latitude (degrees) at the center of the base layer image.

        """
        return self._lon0

    @property
    def res_x(self):
        return self.ds.RasterXSize

    @property
    def res_y(self):
        return self.ds.RasterYSize

    @property
    def image_cs(self):
        """Return image coordinate system.

        """
        if self._image_cs is None:
            self._image_cs = osr.SpatialReference()
            self._image_cs.ImportFromWkt(self._ds.GetProjectionRef())

        return self._image_cs

    @property
    def image(self):
        """Return Numpy image.

        """
        if self._image is None:
            if self.ds.RasterCount > 1:
                self._image = np.zeros((self.res_y,self.res_x,3),
                                       dtype=np.uint8)
                for i in range(3):
                    self._image[:,:,i] = self.ds.GetRasterBand(i+1).ReadAsArray()
            else:
                self._image = self.ds.GetRasterBand(1).ReadAsArray()

        return self._image

    @image.setter
    def image(self, image):
        """Return Numpy image.

        """
        if image.ndim == 2:
            channels = 1
        else:
            channels = image.shape[2]

        if (image.shape[0] == self.res_y and image.shape[1] == self.res_x and
            channels == self.ds.RasterCount):
            # Write the Numpy image to the GDAL raster array.
            if image.ndim == 2:
                self.ds.GetRasterBand(1).WriteArray(image[:,:], 0, 0)
            else:
                for i in range(image.ndim):
                    self.ds.GetRasterBand(i+1).WriteArray(image[:,:,i], 0, 0)

            self._image = None
        else:
            raise Exception('Base layer image update must match the original '
                            'image resolution and channels')

    def lon_lat_from_image_coords(self, im_pt):
        """Return latitude and longitude for image point.

        :param pos: Raw image coordinates of the geotiff that were clicked.
        :type pos: 2-array

        :return: Longitude (degrees) and latitude (degrees) associated with
            the clicked point.
        :rtype: 3-array

        """
        # Convert from image coordinates to the coordinates of the projection
        # encoded in the geotiff.
        Xp,Yp = np.dot(self.geo_tform_mat, [im_pt[0],im_pt[1],1])
        lon,lat,_ = self._to_lla_tform.TransformPoint(Xp, Yp)
        return lon,lat

    def image_coords_from_lon_lat(self, lon, lat):
        """Return image coordinates for latitude and longitude.

        :param lon: Longitude (degrees).
        :type lon: float

        :param lat: Latitude (degrees).
        :type lat: float

        :return: Raw image coordinates of the geotiff.
        :rtype: 2-array

        """
        Xp,Yp,_ = self._from_lla_tform.TransformPoint(lon, lat)

        # Convert from image coordinates to the coordinates of the projection
        # encoded in the geotiff.
        im_pt = np.dot(self.geo_inv_tform_mat, [Xp,Yp,1])
        return im_pt

    def meters_from_image_coords(self, im_pt):
        """Return easting and northing (meters) relative to the image center.

        """
        lon,lat = self.lon_lat_from_image_coords(im_pt)
        enu = cc.llh_to_enu(lat, lon, 0, self.lat0, self.lon0, 0, in_degrees=True)
        return enu[:2]

    def meters_from_lon_lat(self, lon, lat):
        """Return easting and northing (meters) relative to the image center.

        """
        enu = cc.llh_to_enu(lat, lon, 0, self.lat0, self.lon0, 0, in_degrees=True)
        return enu[:2]

    def image_coords_from_meters(self, en):
        """Return image coordinates for easting and northing (meters).

        Easting and northing are measured in a local-level east, north, up
        Cartesian coordinate system with origin at the center of the image.

        """
        lat,lon = cc.enu_to_llh(en[0], en[1], 0, self.lat0, self.lon0, 0,
                             in_degrees=True)[:2]
        im_pt = self.image_coords_from_lon_lat(lon, lat)
        return im_pt

    def resample(self, ul=None, lr=None, scale=(1.0, 1.0), update_origin=False):
        """Resample the base layer to a new discretization.

        :param ul: The (col, row) coordinates of the current image that will be
            mapped to the upper-left corner, (0, 0), coordinates in the
            resampled image.
        :type ul: 2-array

        :param lr: The (col, row) coordinates of the current image that will be
            mapped to the lower-right corner of the resampled image.
        :type lr: 2-array

        :param scale: The (x_scale, y_scale) Scale factors applied
            during resampling. One pixel in the current image will
            span 'scale' pixels in the resampled image.  Values of
            scale less than 1 correspond to downsampling.
        :type downsample: float

        :param update_origin: Update the origin of the easting/northing
            coordinate system to be centered at the new center of the image.
        :type update_origin: bool

        """
        if ul is None:
            ul = (0,0)

        if lr is None:
            lr = (self.res_x,self.res_y)

        x_scale, y_scale = scale

        # Perform the projection/resampling
        gdal_drv = gdal.GetDriverByName('MEM')
        x_size = int(np.round((lr[0] - ul[0])*x_scale))
        y_size = int(np.round((lr[1] - ul[1])*y_scale))
        dest_ds = gdal_drv.Create('', x_size, y_size, self.ds.RasterCount,
                                  gdal.GDT_Byte)
        dest_ds.SetProjection(self.ds.GetProjection())
        geotrans = list(self.ds.GetGeoTransform())
        geotrans[0] += geotrans[1]*ul[0] + geotrans[2]*ul[1]
        geotrans[3] += geotrans[4]*ul[0] + geotrans[5]*ul[1]

        # TODO: Check the math here
        geotrans[1] /= x_scale
        geotrans[2] /= (x_scale / y_scale)
        geotrans[4] /= (y_scale / x_scale)
        geotrans[5] /= y_scale
        dest_ds.SetGeoTransform(geotrans)
        gdal.ReprojectImage(self.ds, dest_ds, self.ds.GetProjection(),
                            dest_ds.GetProjection(), gdal.GRA_Bilinear, 0)

        lat0,lon0 = self.lat0, self.lon0
        self.ds = dest_ds
        if not update_origin:
            self._lat0, self._lon0 = lat0,lon0

    def convert_to_grayscale(self):
        if self.ds.RasterCount == 1:
            raise Exception('Base layer is already grayscale.')

        gdal_drv = gdal.GetDriverByName('MEM')
        dest_ds = gdal_drv.Create('', self.res_x, self.res_y, 1, gdal.GDT_Byte)

        dest_ds.SetGeoTransform(self.ds.GetGeoTransform())
        dest_ds.SetProjection(self.ds.GetProjection())
        new_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self._ds = dest_ds
        self.image = new_image

    def warp_matrix(self, coordinates='meters'):
        """Return matrix that warps from world coordinates to image coordinates.

        :param coordinates: Coordinates to show on the axes.
        :type coordinates: str 'lonlat' | 'meters'

        """
        if coordinates == 'meters':
            res_x = self.res_x
            res_y = self.res_y
            img_coords = np.array([[0,0],[res_x,0],[res_x,res_y],[0,res_y]],
                            dtype=np.float64)
            xy_coords = np.zeros((4,2))
            for i in range(4):
                xy_coords[i] = self.meters_from_image_coords(img_coords[i])

            H = cv2.getPerspectiveTransform(xy_coords.astype(np.float32),
                                            img_coords.astype(np.float32))

        return H

    @property
    def extent_meters(self):
        """Bounds of the full base layer image.

        The output is formatted so that it can be used as the extent parameter
        in maplotlib imshow.

        """
        x1,y1 = self.meters_from_image_coords([0,0])
        x2,y2 = self.meters_from_image_coords([self.res_x,self.res_y])

        extent = [x1,x2,y2,y1]
        return extent

    @property
    def lon_lat_corners(self):
        """Longitude and latitude (degrees) for the four corners of the image.

        corner1 - upper-left corner
        corner2 - upper-right corner
        corner3 - lower-right corner
        corner4 - lower-left corner

        :return: Return longitude and latitude (degrees) for each of the four
            corners.

        """

        res_x = self.res_x
        res_y = self.res_y
        bbox = np.array([[0,0],[res_x,0],[res_x,res_y],[0,res_y]],
                        dtype=np.float64)
        ret = np.zeros((4,2))
        for i in range(4):
            ret[i] = self.lon_lat_from_image_coords(bbox[i])

        return ret

    @property
    def gsd(self):
        """Return average GSD along the horizontal and vertical directions.

        """
        if self._gsd is None:
            ul = self.meters_from_image_coords([0,0])
            lr = self.meters_from_image_coords([self.res_x,self.res_y])
            self._gsd = (abs(ul[0] - lr[0])/self.res_x,
                         abs(ul[1] - lr[1])/self.res_y)

        return self._gsd
