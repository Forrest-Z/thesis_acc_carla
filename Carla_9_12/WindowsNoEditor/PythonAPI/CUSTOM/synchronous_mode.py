#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import print_function
import time
import signal
import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from misc import draw_waypoints
import collections

from acc import ACC
from acc_agent import AccAgent

import matplotlib.pyplot as plt
import datetime
from queue import Queue
from queue import Empty
import csv

class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world,tm, *sensors, **kwargs):
        self.world = world
        self.tm = tm
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data.frame, sensor_name))


def radar_callback(sensor_data: carla.RadarMeasurement, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    #print(sensor_data)
    sensor_queue.put((sensor_data, sensor_data.frame, sensor_name))

def main():
    throttle_probes = []
    brake_probes = []
    velocities = []
    steering = []
    gears = []
    time_probes = []
    time_probes_const = []
    sensor_list = []
    ctrl = []

    i = 0

    actor_list = []
    npc_actor_list = []
    npc_agents = []

    sensor_queue = Queue()
    radar_queue = Queue()

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    tm = client.get_trafficmanager()

    spawn_index = 0


    try:

        m = world.get_map()
        spawn_points = m.get_spawn_points()
        start_pose = spawn_points[spawn_index]
        start_pose.location += carla.Location(5,0,0)
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()
        vehicles = blueprint_library.filter('vehicle.*')
        vehicle_bp = vehicles.find('vehicle.audi.tt')
        radar_bp = blueprint_library.find('sensor.other.radar')

        spawn_points = world.get_map().get_spawn_points()

        for vehicle in vehicles:
            #print(vehicle)
            pass

        for spawn_point in spawn_points:
            print(spawn_point)
            pass

        transform = start_pose
        #vehicle_bp = vehicles.find('vehicle.mercedes.coupe_2020')

        npc_spawn = carla.Transform(transform.location,transform.rotation)

        #print(transform, npc_spawn)
        #ADDING 1 NPC
        #npc = world.spawn_actor(vehicle_bp, npc_spawn)
        #npc_actor_list.append(npc)
        #npc_agents.append(BehaviorAgent(npc))
        #npc_agent = npc_agents[len(npc_agents)-1]

        #start = datetime.datetime.now()
        #while True:
        #    npc.apply_control(npc_agent.run_step())
        #    dt = datetime.datetime.now() -start
        #    dt= dt.seconds
        #    if dt >= 5:
        #        break

        player = world.spawn_actor(vehicle_bp, transform)
        actor_list.append(player)
        player_agent = BehaviorAgent(player)
        dest = start_pose

        bound_x = 0.5 + player.bounding_box.extent.x
        bound_y = 0.5 + player.bounding_box.extent.y
        bound_z = 0.5 + player.bounding_box.extent.z
        radar01 = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=bound_x + 0.01, z=bound_z-0.1),carla.Rotation(pitch=5)), attach_to=player)

        radar01.listen(lambda data: radar_callback(data, radar_queue, "radar01"))
        sensor_list.append(radar01)

        acc = ACC()
        print(dest)

        start_time = datetime.datetime.now()

        #player_agent.set_destination(dest + carla.Location(100,0,0))
        for actor in npc_actor_list:
            actor.set_simulate_physics(True)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, tm,fps=200) as sync_mode:
            t = 0
            time_prev = datetime.datetime.now()
            while True:

                # Advance the simulation and wait for the data.
                sync_mode.tick(timeout=2.0)

                #step all npcs
                for i in range(len(npc_actor_list)):
                    npc = npc_actor_list[i]
                    npc_agent = npc_agents[i]
                    npc.apply_control(npc_agent.run_step())

                #player velocity
                v3 = player.get_velocity()
                velocity = math.sqrt(v3.x ** 2 + v3.y ** 2 + v3.z ** 2)
                radar_vels = []
                #sensors
                try:
                    for _ in range(len(sensor_list)):

                        s_frame = radar_queue.get(True, 1.0)

                        #RADAR CODE
                        radar_data = s_frame[0]
                        current_rot = radar_data.transform.rotation

                        world.debug.draw_point(
                            radar_data.transform.location,
                            size=0.1,
                            life_time=0.06,
                            persistent_lines=False,
                            color=carla.Color(0, 0, 0))

                        for detect in radar_data:
                            #discard background
                            if abs(velocity-acc.radar_velocity_threshold) < abs(detect.velocity) < velocity + acc.radar_velocity_threshold:
                                continue

                            radar_vels.append(detect.velocity + velocity)
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
                            world.debug.draw_point(
                                radar_data.transform.location + fw_vec,
                                size=0.075,
                                life_time=0.06,
                                persistent_lines=False,
                                color=carla.Color(r, g, b))
                            #END OF RADAR CODE

                except Empty:
                    print("    Some of the sensor information is missed")

                control = player_agent.run_step()

                acc.update(velocity)
                control.throttle = acc.control.throttle
                control.brake = acc.control.brake

                dt = datetime.datetime.now() - time_prev
                t += int(dt.microseconds/1000)/1000.0
                time_prev = datetime.datetime.now()
                print(t)
                if t > 10:
                    control.throttle = 1.0
                    control.brake = 0.0

                if t > 80:
                    control.throttle = 0.0
                    control.brake = 1.0

                if t > 100:
                    print("DONE --------")
                    break;


                player.apply_control(control)
                final_control = player.get_control()
                i += 1

                throttle_probes.append(final_control.throttle)
                brake_probes.append(final_control.brake)
                velocities.append(velocity)
                steering.append(final_control.steer)
                gears.append(final_control.gear)
                ctrl.append(acc.pid.out)
                time_probes.append(t)
                time_probes_const.append(sync_mode.delta_seconds*i)


                #print(radar_vels)
                #print("V: {v:.3f} | Throttle: {th:.3f} | Brake: {br:.3f} | CTRL: {ctrl:.3f} | Gear{gr:d}".format(
                #    v=velocity,th=final_control.throttle,br=final_control.brake,ctrl=acc.pid.out,gr=final_control.gear))

    finally:
        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        for actor in npc_actor_list:
            actor.destroy()
        print('done.')

        for sensor in sensor_list:
            sensor.destroy()

        plt.subplot(2,2,1)
        plt.plot(time_probes, velocities)
        plt.title("Velocity")
        #plt.show()

        plt.subplot(2, 2, 2)
        plt.plot(time_probes, gears)
        plt.title("Gear")
        #plt.show()

        plt.subplot(2, 2, 3)
        plt.plot(time_probes, ctrl)
        plt.title("Control")

        plt.show()

        #with open('vel.csv', 'w') as f:
        #    print("writing vel")
        #    write = csv.writer(f)
        #    write.writerow(velocities)
        #with open('brake.csv', 'w') as f:
        #    print("writing brake")
        #    write = csv.writer(f)
        #    write.writerow(brake_probes)
        #with open('throttle.csv', 'w') as f:
        #    print("writing throttle")
        #    write = csv.writer(f)
        #    write.writerow(throttle_probes)
        #with open('time.csv', 'w') as f:
        #    print("writing time")
        #    write = csv.writer(f)
        #    write.writerow(time_probes)
        #with open('time.csv', 'w') as f:
        #   print("writing time")
        #   write = csv.writer(f)
        #   write.writerow(time_probes)
        #with open('timeConst.csv', 'w') as f:
        #   print("writing timeC")
        #   write = csv.writer(f)
        #   write.writerow(time_probes_const)









if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
