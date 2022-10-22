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


class ObstacleParameterization:
    segment_list = {}
    min_width = 0
    viz = {}
    current_s = 0
    tl = {}
    polygons = []
    parameter = {}

    def getParameterisation(self, interpolation:Interpolation, k, s_step):
        s = self.current_s + k * s_step
        XY = interpolation.evaluateInterpolation(s)
        norm  = interpolation.getNormalVectorOfHeadingSingle(s)
        # draw line from point s to track boundaries
        line_width = self.min_width/2 - self.parameter['mpc']['safety_distance']
        point1 = [XY[0] + norm[0]*line_width, XY[1] + norm[1]*line_width]
        point2 = [XY[0] - norm[0]*line_width, XY[1] - norm[1]*line_width]
        shapely_line = LineString([point1, point2])
        widths = []
        e_obs_list = []
        points = [point1, point2]
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

    # def getParameterisation(self, interpolation:Interpolation, k, s_step):
    #     s = self.current_s + k * s_step

    #     XY = interpolation.evaluateInterpolation(s)
    #     norm  = interpolation.getNormalVectorOfHeadingSingle(s)

    #     point1 = [XY[0] + norm[0]*self.min_width/2, XY[1] + norm[1]*self.min_width/2]
    #     point2 = [XY[0] - norm[0]*self.min_width/2, XY[1] - norm[1]*self.min_width/2]
    #     shapely_line = LineString([point1, point2])
        
    #     poly = {}
    #     widths = []
    #     e_obs_list = []
    #     points = [point1, point2]
    #     self.viz.visualize_obstacle_line(points)

    #     max_dist = self.parameter['mpc']['n_horizon'] * self.parameter['mpc']['t_step']
    #     for segment in self.segment_list:
    #         if segment is None or len(segment.points) < 3:
    #             continue
            
    #         close = False

    #         stampedPoints = []
    #         for pt in segment.points:
    #             stamped = PointStamped()
    #             stamped.header.frame_id = "ego_racecar/base_link"
    #             stamped.point = pt
    #             stampedPoints.append(stamped)

    #             if not close and np.sqrt(pt.x**2 + pt.y**2).real < max_dist:
    #                 print('Polygon is close')
    #                 close = True

    #         if close:

    #             poly = [[self.tl.transformPoint("map", point).point.x, self.tl.transformPoint("map", point).point.y] for point in stampedPoints]
    #             shapely_poly = Polygon(poly)

    #             minx = shapely_poly.bounds[0]
    #             miny = shapely_poly.bounds[1]
    #             maxx = shapely_poly.bounds[2]
    #             maxy = shapely_poly.bounds[3]
    #             boundingBox = [[minx, miny],[minx, maxy],[maxx, maxy],[maxx, miny]]
    #             shapely_poly = Polygon(boundingBox)

    #             self.viz.visualize_obstacle_poly(poly)
    #             self.viz.visualize_obstacle_poly(boundingBox)
    #             try:
    #                 intersection = shapely_poly.intersection(shapely_line)
    #                 mp = intersection.boundary
    #                 if len(mp.geoms) == 0:
    #                     continue
    #                 intersection_line = [[mp.geoms[0].x, mp.geoms[0].y], [mp.geoms[len(mp.geoms)-1].x, mp.geoms[len(mp.geoms)-1].y]]
    #             except shapely.errors.TopologicalError:
    #                 # print("WARNING: Invalid geometry on polygon led to error in obstacle-parameterization:")
    #                 # print(poly)
    #                 continue

    #             if len(intersection_line) == 0:
    #                 continue

    #             # print("OBSTACLE FOUND IN PATH")
    #             distances =  [interpolation.calculateNormDistance(intersection_line[0], s), interpolation.calculateNormDistance(intersection_line[1], s)]

    #             width = np.max(distances) - np.min(distances)
    #             e_obs = np.min(distances) + width/2
    #             # print("e_obs: {obs}, width: {wid}".format(obs = e_obs, wid = width))
    #             self.viz.visualize_obstacle_param(interpolation, s, e_obs, width)
    #             widths.append(width)
    #             e_obs_list.append(e_obs)

    #     return widths, e_obs_list

