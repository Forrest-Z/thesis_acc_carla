from __future__ import print_function

import signal
import time
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref


try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

from pid import PID

#################################################################################################
from luts import LUT_brake_distance


class ACC:
    def __init__(self,world):
        print("Creating ACC obj")
        self.radar_velocity_threshold = 0.5 #0.278 m/s = 1km/h
        self.setpoint_velocity = 25

        self.pid = PID()
        self.pid_distance = PID()
        self.control = carla.VehicleControl()
        self.world = world
        self.lut_brake = LUT_brake_distance()
        self.detected_vel = []
        self.detected_dst = []
        self.radar_time = datetime.datetime.now()
        self.radar_timeout = 25000  # u sec= 1sec
        self.radar_dt = self.radar_timeout + 1
        self.distanceInit = False
        pass

    def update(self, measured_velocity, radar_data, current_rot=carla.Rotation(0, 0, 0)):
        self.radar_dt = datetime.datetime.now() - self.radar_time

        #print("radar dt", self.radar_dt)

        if not self.detected_vel and self.radar_dt.microseconds > self.radar_timeout:
            print("RESET RADAR DATA")
            self.detected_vel.clear()
            self.detected_dst.clear()

        self.radar_ack(measured_velocity, radar_data, current_rot)
        u = 0
        #print(len(self.detected_vel))
            #or min(self.detected_vel) > 0
        distance_multi = measured_velocity
        distance_multi = 3
        """if self.detected_vel:
            u = self.pid.step(min(self.detected_vel) * (min(self.detected_dst)), measured_velocity)
        elif not self.detected_vel:
            u = self.pid.step(self.setpoint_velocity, measured_velocity)"""

        #SORTA WORKING
        """if not self.detected_vel or min(self.detected_dst) > self.lut_brake.get_distance(measured_velocity) * distance_multi:
        #if not self.detected_vel or min(self.detected_dst) > 100:
        #if not self.detected_vel:
            self.distanceInit = False
            print("PID V")
            dst = -1
            if self.detected_dst:
                dst = min(self.detected_dst)
            #print("Distance: {dstmin:.3f} | Target vel: {v:.3f} |  Braking distance: {braking:.3f}".format(dstmin=dst, v=measured_velocity, braking= self.lut_brake.get_distance(measured_velocity)))
            u = self.pid.step(self.setpoint_velocity, measured_velocity)

        else:
            #print("PID DISTANCE")
            if not self.distanceInit:
                #self.pid_distance.limMax = self.pid.out
                self.pid_distance.proportional = self.pid.proportional
                self.pid_distance.differentiator = self.pid.differentiator
                self.pid_distance.integrator = self.pid.integrator
                self.pid_distance.out = self.pid.out
                self.pid_distance.prev_measurement = min(self.detected_dst)
                self.pid_distance.prev_error = min(self.detected_dst)
                self.distanceInit = True

            dst = min(self.detected_dst)
            vel = measured_velocity + min(self.detected_vel)

            braking_dst = self.lut_brake.get_distance(vel) * distance_multi

            #self.pid_distance.Kp = -0.2653
            #self.pid_distance.Ki = -0.003576
            #self.pid_distance.Kd = -4.921

            self.pid_distance.Kp = -0.2653 * 1000
            self.pid_distance.Ki = -0.003576
            self.pid_distance.Kd = -4.921
            self.pid_distance.tau = 0.1
            print("Distance: {dstmin:.3f} | Vel:{v:.3f} | Target vel: {tv:.3f} |  Braking distance: {braking:.3f}".format(dstmin=dst, v=measured_velocity,tv=vel, braking=braking_dst))
            u = self.pid_distance.step(braking_dst, dst)"""
        u1 = 0
        u2 = 0

        if not self.detected_vel:
            u1 = self.pid.step(self.setpoint_velocity, measured_velocity)
        else:
            #print("dist control")
            u1 = self.pid.step(min(self.detected_vel)+measured_velocity, measured_velocity)

            dst = min(self.detected_dst)
            vel = measured_velocity + min(self.detected_vel)

            braking_dst = self.lut_brake.get_distance(vel)
            braking_dst = 5
            #self.pid_distance.Kp = -0.2653
            #self.pid_distance.Ki = -0.003576
            #self.pid_distance.Kd = -4.921
            #self.pid_distance.tau = 0.1

            #self.pid_distance.Kp = 0.08
            #self.pid_distance.Ki = 0.1
            #self.pid_distance.Kd = 10.100
            #self.pid_distance.tau = 0.1

            self.pid_distance.limMax = 2
            print("Distance: {dstmin:.3f} | Vel:{v:.3f} | Target vel: {tv:.3f} |  Braking distance: {braking:.3f}".format(dstmin=dst, v=measured_velocity, tv=vel, braking=braking_dst))
            u2 = self.pid_distance.step(-braking_dst, -dst)

        u = u1 + u2
        if u > 1:
            u = 1
        print("u: ", u ,"| u1: ", u1," | u2: ", u2)


        #print("u: {ctrl:.3f}".format(ctrl=u))
        if u > 0:
            self.control.throttle = u
            self.control.brake = 0
        else:
            self.control.throttle = 0
            self.control.brake = u
        pass

    def radar_ack(self, velocity, radar_data, current_rot):

        braking_distance = self.lut_brake.get_distance(velocity)
        #print("Breaking distance: {v:.2f}".format(v=braking_distance))
        min_detect = 1000.0
        max_detect = -1000.0
        min_distance = 1000.0

        if radar_data:
            #print("Got valid radar data resetting t/o")
            #print("LEN:", len(radar_data))

            self.detected_vel.clear()
            self.detected_dst.clear()

        for detect in radar_data:
            # discard background
            if abs(velocity - self.radar_velocity_threshold) < abs(detect.velocity) < abs(velocity + self.radar_velocity_threshold):
                continue
            if abs(velocity) < self.radar_velocity_threshold*2:
                continue

            self.radar_time = datetime.datetime.now()
            self.radar_dt = datetime.datetime.now() - datetime.datetime.now()

            self.detected_vel.append(detect.velocity)
            self.detected_dst.append(detect.depth)

            if detect.velocity < min_detect:
                min_detect = detect.velocity
            if detect.velocity > max_detect:
                max_detect = detect.velocity
            if detect.depth < min_distance:
                min_distance = detect.depth

            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            velocity_range = 10  # m/s
            norm_velocity = detect.velocity / velocity_range  # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))
            # END OF RADAR LOOP

            #print(self.detected_dst)
        pass
    pass
