from distutils.command.config import config
from unittest import skip
from time_optimal_mpc.utils.interpolation import Interpolation
from numpy import poly
from shapely.geometry import Polygon
from shapely.geometry import LineString
from shapely.geometry import MultiPoint
from tf import TransformListener
from geometry_msgs.msg import PointStamped
import numpy
import numpy as np
import shapely
from time_optimal_mpc.utils import config


class ObstacleParameterization:
    segment_list = {}
    min_width = 0
    viz = {}
    current_s = 0
    tl = {}
    polygons = []
    config = {}

    def getParameterisation(self, interpolation:Interpolation, s):

        XY = interpolation.evaluateInterpolation(s)
        norm  = interpolation.getNormalVectorOfHeadingSingle(s)
        # draw line from point s to track boundaries
        line_width = self.min_width/2 - config.safety_distance
        point1 = [XY[0] + norm[0]*line_width, XY[1] + norm[1]*line_width]
        point2 = [XY[0] - norm[0]*line_width, XY[1] - norm[1]*line_width]
        shapely_line = LineString([point1, point2])
        widths = []
        e_obs_list = []
        points = [point1, point2]

        # check intersections of right angle lines with obstacles
        for poly in self.polygons:
            try:
                intersection = poly.intersection(shapely_line)
                mp = intersection.boundary
                if len(mp.geoms) == 0:
                    continue
                intersection_line = [[mp.geoms[0].x, mp.geoms[0].y], [mp.geoms[len(mp.geoms)-1].x, mp.geoms[len(mp.geoms)-1].y]]
            except shapely.errors.TopologicalError:
                continue
            if len(intersection_line) == 0:
                continue
            distances =  [interpolation.calculateNormDistance(intersection_line[0], s), interpolation.calculateNormDistance(intersection_line[1], s)]
            width = np.max(distances) - np.min(distances)
            e_obs = np.min(distances) + width/2
            widths.append(width)
            e_obs_list.append(e_obs)
        return widths, e_obs_list

