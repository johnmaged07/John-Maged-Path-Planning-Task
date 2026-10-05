from __future__ import annotations

import math
from typing import List

from src.models import CarPose, Cone, Path2D


HALF_WIDTH = 1.0         
PATH_LENGTH = 8.0        
STEP = 0.25              
SMOOTHING_PASSES = 15   


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(dx, dy):
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return (1.0, 0.0)
    return (dx / length, dy / length)


def sort_points(points, numbers):
    pairs = []
    for k in range(len(points)):
        pairs.append((numbers[k], points[k]))
    pairs.sort()
    result = []
    for number, point in pairs:
        result.append(point)
    return result


def sort_along(points, direction):
    numbers = []
    for p in points:
        numbers.append(p[0] * direction[0] + p[1] * direction[1])
    return sort_points(points, numbers)


def nearest_first(points, car):
    numbers = []
    for p in points:
        numbers.append(distance(p, car))
    return sort_points(points, numbers)


class PathPlanning:
    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        car = (self.car_pose.x, self.car_pose.y)
        heading = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))


        blue = []
        yellow = []
        for cone in self.cones:
            if cone.color == 1:
                blue.append((cone.x, cone.y))
            elif cone.color == 0:
                yellow.append((cone.x, cone.y))


        if len(blue) > 0 and len(yellow) > 0:
            anchors, end_dir = self._both_sides(blue, yellow)
        elif len(blue) > 0:
            anchors, end_dir = self._one_side(nearest_first(blue, car), True, heading)
        elif len(yellow) > 0:
            anchors, end_dir = self._one_side(nearest_first(yellow, car), False, heading)
        else:
            anchors, end_dir = [], heading          

        ahead = []
        for a in anchors:
            dx = a[0] - car[0]
            dy = a[1] - car[1]
            if dx * end_dir[0] + dy * end_dir[1] > -0.5 or dx * heading[0] + dy * heading[1] > -0.5:
                ahead.append(a)

        last = car
        if len(ahead) > 0:
            last = ahead[-1]
        far_point = (last[0] + end_dir[0] * PATH_LENGTH, last[1] + end_dir[1] * PATH_LENGTH)
        points = self._resample([car] + ahead + [far_point])


        points = self._keep_clear(points, blue, yellow)
        for _ in range(SMOOTHING_PASSES):           
            new_points = [points[0]]               
            for n in range(1, len(points) - 1):
                x = (points[n - 1][0] + points[n][0] + points[n + 1][0]) / 3
                y = (points[n - 1][1] + points[n][1] + points[n + 1][1]) / 3
                new_points.append((x, y))
            new_points.append(points[-1])
            points = new_points
        return points


    def _both_sides(self, blue, yellow):
        b0 = blue[0]
        y0 = yellow[0]
        for b in blue:
            for y in yellow:
                if distance(b, y) < distance(b0, y0):
                    b0 = b
                    y0 = y
        track_dir = unit(-(y0[1] - b0[1]), y0[0] - b0[0])


        blue = sort_along(blue, track_dir)
        yellow = sort_along(yellow, track_dir)

        i = 0                                       
        j = 0                                       
        edges = [(blue[0], yellow[0])]
        while i < len(blue) - 1 or j < len(yellow) - 1:
            if i == len(blue) - 1:
                j = j + 1                           
            elif j == len(yellow) - 1:
                i = i + 1                           
            elif distance(blue[i + 1], yellow[j]) <= distance(blue[i], yellow[j + 1]):
                i = i + 1                           
            else:
                j = j + 1                           
            edges.append((blue[i], yellow[j]))

        anchors = []
        for b, y in edges:
            anchors.append(((b[0] + y[0]) / 2, (b[1] + y[1]) / 2))

        end_dir = track_dir
        if len(anchors) >= 2:
            end_dir = unit(anchors[-1][0] - anchors[-2][0], anchors[-1][1] - anchors[-2][1])
        return anchors, end_dir

    def _one_side(self, cones, is_blue, heading):
        shift = HALF_WIDTH
        if is_blue:
            shift = -HALF_WIDTH

        anchors = []
        direction = heading                        
        for k in range(len(cones)):
            if len(cones) > 1:
                before = cones[max(k - 1, 0)]                   
                after = cones[min(k + 1, len(cones) - 1)]     
                direction = unit(after[0] - before[0], after[1] - before[1])
            left_x = -direction[1]                
            left_y = direction[0]
            anchors.append((cones[k][0] + left_x * shift, cones[k][1] + left_y * shift))
        return anchors, direction

    def _keep_clear(self, points, blue, yellow):
        rooms = []
        for cones, others in [(blue, yellow), (yellow, blue)]:
            for cone in cones:
                room = HALF_WIDTH
                for other in others:
                    room = min(room, distance(cone, other) / 2)
                rooms.append((cone, room))

        for _ in range(3):                          
            for n in range(1, len(points)):       
                x, y = points[n]                    
                for cone, room in rooms:
                    d = distance((x, y), cone)
                    if 0 < d < room:
                        x = cone[0] + (x - cone[0]) * room / d
                        y = cone[1] + (y - cone[1]) * room / d
                points[n] = (x, y)
        return points

    def _resample(self, line):
        max_points = int(PATH_LENGTH / STEP) + 1
        points = [line[0]]
        next_at = STEP                           
        travelled = 0.0                            
        for n in range(len(line) - 1):
            length = distance(line[n], line[n + 1])
            while next_at <= travelled + length and len(points) < max_points:
                t = (next_at - travelled) / length  # 0 = start of this piece, 1 = end of it
                points.append((line[n][0] + (line[n + 1][0] - line[n][0]) * t,
                               line[n][1] + (line[n + 1][1] - line[n][1]) * t))
                next_at = next_at + STEP
            travelled = travelled + length
        return points