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

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
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


def main():

    def add_bot(spawn_point:carla.Transform, vehicle_bp, move_time=0, offset:carla.Location= carla.Location(0, 0, 0)):
        spawn_point.location = spawn_point.location + offset
        bot = world.spawn_actor(vehicle_bp, spawn_point)
        bot.set_simulate_physics(True)

        bot_agent = BehaviorAgent(bot)

        bot_actors.append(bot)
        bot_agents.append(bot_agent)

        start_time = datetime.datetime.now()
        dt = datetime.datetime.now() - start_time

        while int(dt.seconds) < int(move_time):
            bot.apply_control(bot_agent.run_step())
            dt = datetime.datetime.now() - start_time
        return bot

    bot_actors = []
    bot_agents = []

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    spawn_index = 0

    try:
        m = world.get_map()
        spawn_points = m.get_spawn_points()
        start_pose = spawn_points[spawn_index]
        spawn_offset = carla.Location(5,0,0)
        start_pose.location = start_pose.location + spawn_offset

        blueprint_library = world.get_blueprint_library()
        vehicle_name = 'vehicle.audi.tt'
        vehicles = blueprint_library.filter('vehicle.*')
        vehicle_bp = vehicles.find('vehicle.audi.tt')

        bot1 = add_bot(spawn_point=start_pose, vehicle_bp=vehicle_bp, move_time=15)
        player_agent = AccAgent(world,vehicle_name,start_pose)


        # Create a synchronous mode context.
        with CarlaSyncMode(world, fps=200) as sync_mode:

            t = 0
            time_prev = datetime.datetime.now()
            while True:

                # Advance the simulation and wait for the data.
                sync_mode.tick(timeout=2.0)
                #time.sleep(0.016)

                #step all npcs
                for i in range(len(bot_actors)):
                    npc = bot_actors[i]
                    npc_agent = bot_agents[i]
                    npc_control = npc_agent.run_step()
                    npc_control.throttle = 1
                    npc_control.reverse = 1
                    npc_control.steer = 0
                    npc.apply_control(npc_control)

                player_agent.update()

                dt = datetime.datetime.now() - time_prev
                t += int(dt.microseconds/1000)/1000.0
                time_prev = datetime.datetime.now()

    finally:

        player_agent.release()

        for npc in bot_actors:
            npc.destroy()
        print('Cleanup finished')


if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
