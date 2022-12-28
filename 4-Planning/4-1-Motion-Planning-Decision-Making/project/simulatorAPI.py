#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

#/**********************************************
# * Self-Driving Car Nano-degree - Udacity
# *  Created on: September 20, 2020
# *      Author: Munir Jojo-Verge
#                Aaron Brown
# **********************************************/



"""
Welcome to CARLA path planning

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


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
import asyncio
import json
from websocket import create_connection

way_points = []
v_points = []
spirals_x = []
spirals_y = []
spirals_v = []
obst_x = []
obst_y = []
spiral_idx = []
last_move_time = -1
update_cycle = True
velocity = 0

_prev_junction_id = -1
_tl_state = 'none'
_active_maneuver = 0
_update_point_thresh = 16
_prev_yaw = 0
_pivot = carla.Transform()

# keep track of how level the ground is for teleportation
_road_height = 0
_road_pitch = 0
_road_roll = 0

# orbital camera
_view_radius = 10
_view_radius_change = 0
_view_yaw = 0
_view_yaw_change = 0
_view_pitch = 0
_view_pitch_change = 0

ws=create_connection("ws://localhost:4567")
       
try:
    import pygame
    from pygame.locals import K_RIGHT
    from pygame.locals import K_LEFT
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_w
    from pygame.locals import K_s
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def magnitude(vector):
    return math.sqrt(vector.x*vector.x+vector.y*vector.y+vector.z*vector.z)

def distance_lookahead(velocity_mag, accel_mag, time=1.5, min_distance=8.0, max_distance=20.0):
    return max( min(velocity_mag * time + 0.5 * accel_mag * time * time, max_distance), min_distance)


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.camera_manager = None
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.spectator = self.world.get_spectator()

    def get_waypoint(self, pos_x, pos_y):
        global _prev_junction_id, _tl_state, _road_height, _road_pitch, _road_roll
        position = carla.Location(x= pos_x, y= pos_y)
        waypoint = self.map.get_waypoint(position, project_to_road=True, lane_type=(carla.LaneType.Driving))
        _road_height = waypoint.transform.location.z
        _road_pitch = waypoint.transform.rotation.pitch
        _road_roll = waypoint.transform.rotation.roll

        if (_active_maneuver == 2 or _active_maneuver == 3):
            point = waypoint.transform.location

            if self.player.is_at_traffic_light():

                 #Change a red traffic light to green
                traffic_light =  self.player.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    traffic_light.set_state(carla.TrafficLightState.Green)
                    traffic_light.set_green_time(4.0)


                _tl_state = str(carla.TrafficLightState.Green)

            return point.x, point.y, waypoint.transform.rotation.yaw * math.pi/180, False
  
        waypoint = waypoint.next(distance_lookahead(magnitude(self.player.get_velocity() ), magnitude(self.player.get_acceleration() ) ))[0]
        point = waypoint.transform.location

        not_junction = [475, 1168, 82, 1932]

        _tl_state = 'none'

        is_goal_junction = waypoint.is_junction
        if (is_goal_junction):
            if self.player.is_at_traffic_light():
                traffic_light = self.player.get_traffic_light()
                _tl_state = str(traffic_light.get_state())
            cur_junction_id = waypoint.get_junction().id
            if (cur_junction_id == _prev_junction_id or cur_junction_id in not_junction or _tl_state is 'Green'):
                is_goal_junction = False;
            else:
                _prev_junction_id = cur_junction_id
  
        return point.x, point.y, waypoint.transform.rotation.yaw * math.pi/180, is_goal_junction

    def move(self, sim_time, res=1):
        global way_points, v_points, last_move_time, velocity, _prev_yaw, _view_yaw, _view_pitch, _view_radius, _pivot
        previous_index = 0
        start = carla.Transform()
        end = carla.Transform()

        # draw spirals
        height_plot_scale = 1.0
        height_plot_offset = 1.0
        blue = carla.Color(r=0, g=0, b=255)
        green = carla.Color(r=0, g=255, b=0)
        red = carla.Color(r=255, g=0, b=0)
        for i in range(len(spirals_x)):
            previous_index = 0
            previous_speed = 0
            start = carla.Transform()
            end = carla.Transform()
            color = blue
            if i == spiral_idx[-1]:
                color = green
            elif i in spiral_idx[:-1]:
                color = red
            for index in range(1, len(spirals_x[i])):
                start.location.x = spirals_x[i][previous_index]
                start.location.y = spirals_y[i][previous_index]
                end.location.x = spirals_x[i][index]
                end.location.y = spirals_y[i][index]
                start.location.z = height_plot_scale * spirals_v[i][previous_index] + height_plot_offset + _road_height
                end.location.z =  height_plot_scale * spirals_v[i][index] + height_plot_offset + _road_height
                self.world.debug.draw_line(start.location, end.location, 0.1, color, .1)
                previous_index = index  

        
        # draw path
        previous_index = 0
        for index in range(res, len(way_points), res):
            start.location = way_points[previous_index].location
            end.location = way_points[index].location
            start.location.z = height_plot_scale * v_points[previous_index] + height_plot_offset + _road_height
            end.location.z = height_plot_scale * v_points[index] + height_plot_offset + _road_height
            self.world.debug.draw_line(start.location, end.location, 0.1, carla.Color(r=125, g=125, b=0), .1)
            previous_index = index
        
        # increase wait time for debug
        wait_time = 0.0
        delta_t = 0.05
        if (sim_time - last_move_time) > wait_time:
            last_move_time = sim_time
            # move car using interpolation based on the velocity label form trajectory points
            if len(way_points) > 1:
                yaw = math.atan2(way_points[1].location.y-way_points[0].location.y, way_points[1].location.x-way_points[0].location.x)
                velocity = v_points[0]
                if velocity < 0.01 or _active_maneuver == 3:
                    yaw = _prev_yaw
                    way_points.pop(0)
                    v_points.pop(0)
                else:
                    _prev_yaw = yaw
                    D = velocity * delta_t
                    d_interval = math.sqrt( (way_points[1].location.x - way_points[0].location.x )**2 + (way_points[1].location.y - way_points[0].location.y )**2  )
                    while d_interval < D and len(way_points) > 2:
                        D -= d_interval
                        way_points.pop(0)
                        v_points.pop(0)
                        d_interval = math.sqrt( (way_points[1].location.x - way_points[0].location.x )**2 + (way_points[1].location.y - way_points[0].location.y )**2  )
                    ratio = D / d_interval
                    v_points[0] = ratio * (v_points[1]-v_points[0]) + v_points[0]
                    way_points[0].location.x = ratio * (way_points[1].location.x - way_points[0].location.x) + way_points[0].location.x
                    way_points[0].location.y = ratio * (way_points[1].location.y - way_points[0].location.y) + way_points[0].location.y
                    yaw = math.atan2(way_points[1].location.y-way_points[0].location.y, way_points[1].location.x-way_points[0].location.x)
                
                
                way_points[0].rotation.yaw = yaw * 180 / math.pi
                way_points[0].rotation.pitch = _road_pitch
                way_points[0].rotation.roll = _road_roll
                way_points[0].location.z = _road_height


                _pivot = carla.Transform()
                _pivot.location = self.player.get_transform().location
                _pivot.location.x += _view_radius * math.cos(math.pi + _view_yaw)
                _pivot.location.y += _view_radius * math.sin(math.pi + _view_yaw)
                _view_yaw += _view_yaw_change
                _view_pitch += _view_pitch_change
                _view_radius += _view_radius_change
                _view_radius = min( max(10, _view_radius), 100)
                while _view_yaw < -math.pi:
                    _view_yaw += math.pi
                while _view_yaw > math.pi:
                    _view_yaw -= math.pi
                while _view_pitch < -math.pi:
                    _view_pitch += math.pi
                while _view_pitch > math.pi:
                    _view_pitch -= math.pi
                _pivot.rotation.yaw  = _view_yaw * 180 / math.pi
                _pivot.rotation.pitch  = _view_pitch * 180 / math.pi
                _pivot.location.z += 2 + _view_radius * math.sin(math.pi + _view_pitch)
                self.player.set_transform(way_points[0])
                

            


    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        #blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint = self.world.get_blueprint_library().filter(self._actor_filter)[12]
        #cars = self.world.get_blueprint_library().filter(self._actor_filter)
        #for car in cars:
        #    print(car)
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            #print(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_point = self.map.get_spawn_points()[1]
            #print(len(spawn_points))
            #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

            '''
            self.actor = self.world.try_spawn_actor(blueprint, actor_spawn)
            '''

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = v_points[0]
        c = world.player.get_control()
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f m/s' % v,
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]

        if _active_maneuver == 0:
            behavior = 'FOLLOW LANE'
        elif _active_maneuver == 1:
            behavior = 'FOLLOW VEHICLE'
        elif _active_maneuver == 2:
            behavior = 'DECEL TO STOP'
        else:
            behavior = 'STOPPED'
        self._info_text += ['Behavior: '+behavior]
        self._info_text += ['Prev Junction: '+str(_prev_junction_id)]
        if _tl_state is not 'none':
            self._info_text += ['TL State: '+_tl_state]
        self._info_text += ['num waypoints: '+str(len(way_points))]
        num_spirals = len(spirals_x)
        spirals_coll = max(len(spiral_idx)-1, 0)
        spirals_free = str(num_spirals - spirals_coll)
        self._info_text += ['Spirals: '+spirals_free+' / '+str(num_spirals)]

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-9.5+parent_actor.get_transform().location.x, y= parent_actor.get_transform().location.y, z=3.5), carla.Rotation(pitch=8.0, yaw= parent_actor.get_transform().rotation.yaw)),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.SpringArm)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

            item.append(bp)
        self.index = None

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index])
                #attach_to=self._parent,
                #attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

        self._camera_transforms[self.transform_index] =_pivot
        self.sensor.set_transform(_pivot)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        
        if self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world):
        print("init keyboard control")

    def parse_events(self, client, world):
        global _view_yaw_change, _view_pitch_change, _view_radius_change

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    _view_yaw_change = 0.01
                elif event.key == pygame.K_RIGHT:
                    _view_yaw_change = -0.01
                if event.key == pygame.K_UP:
                    _view_pitch_change = -0.01
                elif event.key == pygame.K_DOWN:
                    _view_pitch_change = 0.01
                if event.key == pygame.K_w:
                    _view_radius_change = 0.2
                elif event.key == pygame.K_s:
                    _view_radius_change = -0.2
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    _view_yaw_change = 0
                if event.key == pygame.K_DOWN or event.key == pygame.K_UP:
                    _view_pitch_change = 0
                if event.key == pygame.K_w or event.key == pygame.K_s:
                    _view_radius_change = 0


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def SpawnNPC(client, world, args, offset_x, offset_y):
    global obst_x, obst_y
    SpawnActor = carla.command.SpawnActor
    blueprint = client.get_world().get_blueprint_library().filter(args.filter)[5]
    actor_spawn =  carla.Transform()
    spawn_point = world.map.get_spawn_points()[1]
    actor_spawn.location = spawn_point.location
    actor_spawn.rotation = spawn_point.rotation
    actor_spawn.location.x += offset_x * spawn_point.get_forward_vector().x
    actor_spawn.location.y += offset_y * spawn_point.get_right_vector().y
    obst_x.append(actor_spawn.location.x)
    obst_y.append(actor_spawn.location.y)


    return SpawnActor(blueprint, actor_spawn)

@asyncio.coroutine
def game_loop(args):
    global update_cycle
    pygame.init()
    pygame.font.init()
    world = None
    vehicles_list = []

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world)

        clock = pygame.time.Clock()
        
        world.player.set_simulate_physics(False)

        # Spawn some obstacles to avoid
        batch = []
        batch.append(SpawnNPC(client, world, args, 30, 1.5))
        batch.append(SpawnNPC(client, world, args, 65, -3 *1.5))
        batch.append(SpawnNPC(client, world, args, 110, -1 * 1.5))

        for response in carla.Client.apply_batch_sync(client, batch):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        start_time = world.hud.simulation_time
        
        while True:
            yield from asyncio.sleep(0.01) # check if any data from the websocket

            if controller.parse_events(client, world):
                return
            
            if len(way_points) == 0:
                player = world.player.get_transform()
                way_points.append(player)
                v_points.append(0)

            
            sim_time = world.hud.simulation_time-start_time
            
            if update_cycle and (len(way_points) < _update_point_thresh):
                
                update_cycle = False
                # print("sending data")

                x_points = [point.location.x for point in way_points]
                y_points = [point.location.y for point in way_points]
                yaw = way_points[0].rotation.yaw * math.pi / 180
                waypoint_x, waypoint_y, waypoint_t, waypoint_j = world.get_waypoint(x_points[-1], y_points[-1])
    
                ws.send(json.dumps({'traj_x': x_points, 'traj_y': y_points, 'traj_v': v_points ,'yaw': yaw, "velocity": velocity, 'time': sim_time, 'waypoint_x': waypoint_x, 'waypoint_y': waypoint_y, 'waypoint_t': waypoint_t, 'waypoint_j': waypoint_j, 'tl_state': _tl_state, 'obst_x': obst_x, 'obst_y': obst_y } ))
               
            
            clock.tick_busy_loop(60)
            world.tick(clock)
            
            world.move(sim_time)
            world.render(display)
            pygame.display.flip()

    finally:

        print("key board interrupt, good bye")
        if world is not None:
            world.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def get_data():
    global way_points, v_points, update_cycle, spirals_x, spirals_y, spirals_v, spiral_idx, _active_maneuver
    data = ws.recv() # blocking call, thats why we use asyncio
    data = json.loads(str(data))
    dist_thresh = 0.1
    closest_dist = 1000
    start_index = 0

    spirals_x = data['spirals_x']
    spirals_y = data['spirals_y']
    spirals_v = data['spirals_v']
    spiral_idx = data['spiral_idx']
    _active_maneuver = data['active_maneuver']

    # Start at the point that is closest to the start way point
    if( len(way_points) > 1):
        for path_index in range(len(data['trajectory_x'])):

            new_x = data['trajectory_x'][path_index]
            new_y = data['trajectory_y'][path_index]
            dist = math.sqrt(math.pow(new_x-way_points[1].location.x, 2)+math.pow(new_y-way_points[1].location.y, 2))
            if dist < dist_thresh:
                start_index = max(path_index-1,0)
                break
            elif dist < closest_dist:
                start_index = max(path_index-1,0)
                closest_dist = dist
            if path_index is len(data['trajectory_x'])-1:
                print("WARNING: distance start threshold not met ", closest_dist)

    #print("start index ", start_index) # test if start_index is moving

    x_set = data['trajectory_x'][start_index:]
    y_set = data['trajectory_y'][start_index:]
    v_set = data['trajectory_v'][start_index:]
    _update_point_thresh = data['update_point_thresh']

    for path_index in range(len(x_set)):
        new_x = x_set[path_index]
        new_y = y_set[path_index]
        new_v = v_set[path_index]

        if path_index >= len(way_points):
            point = carla.Transform()
            point.location.x = new_x
            point.location.y = new_y
            way_points.append(point)
            v_points.append(new_v)
        elif (new_x != way_points[path_index].location.x) or (new_y != way_points[path_index].location.y):
            way_points[path_index].location.x = new_x
            way_points[path_index].location.y = new_y
            v_points[path_index] = new_v

    update_cycle = True


@asyncio.coroutine
def ws_event(loop):
    while True:
        yield from loop.run_in_executor(None, get_data)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    #sio.connect('http://localhost:4567')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(
        asyncio.wait([
            ws_event(loop),
            game_loop(args)
        ])
    )

if __name__ == '__main__':

    main()
