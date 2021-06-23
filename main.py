#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
import sys

try:
	sys.path.append(glob.glob('carla/dist/carla-*%d.%d-%s.egg' % (
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
	import pygame
	from pygame.locals import KMOD_CTRL
	from pygame.locals import KMOD_SHIFT
	from pygame.locals import K_0
	from pygame.locals import K_9
	from pygame.locals import K_BACKQUOTE
	from pygame.locals import K_BACKSPACE
	from pygame.locals import K_COMMA
	from pygame.locals import K_DOWN
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_F1
	from pygame.locals import K_LEFT
	from pygame.locals import K_PERIOD
	from pygame.locals import K_RIGHT
	from pygame.locals import K_SLASH
	from pygame.locals import K_SPACE
	from pygame.locals import K_TAB
	from pygame.locals import K_UP
	from pygame.locals import K_a
	from pygame.locals import K_b
	from pygame.locals import K_c
	from pygame.locals import K_d
	from pygame.locals import K_g
	from pygame.locals import K_h
	from pygame.locals import K_i
	from pygame.locals import K_l
	from pygame.locals import K_m
	from pygame.locals import K_n
	from pygame.locals import K_p
	from pygame.locals import K_q
	from pygame.locals import K_r
	from pygame.locals import K_s
	from pygame.locals import K_v
	from pygame.locals import K_w
	from pygame.locals import K_x
	from pygame.locals import K_z
	from pygame.locals import K_MINUS
	from pygame.locals import K_EQUALS
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
	import numpy as np
except ImportError:
	raise RuntimeError('cannot import numpy, make sure numpy package is installed')

#my imports
from receiver        import *
from sender          import *
from argsParser      import *
from multiprocessing import Queue
import threading
import json

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
def find_weather_presets():
	rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
	name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
	presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
	return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
	name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
	return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
	def __init__(self, carla_world, hud, args, qForSendingImuDataToBalancerModule):
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
		self.qForSendingImuDataToBalancerModule = qForSendingImuDataToBalancerModule
		self.imu_sensor = None
		self.camera_manager = None
		self._weather_presets = find_weather_presets()
		self._weather_index = 0
		self._actor_filter = args.filter
		self._gamma = args.gamma
		self.restart()
		self.world.on_tick(hud.on_world_tick)
		self.recording_enabled = False
		self.recording_start = 0
		self.constant_velocity_enabled = False
		self.current_map_layer = 0
		self.map_layer_names = [
			carla.MapLayer.NONE,
			carla.MapLayer.Buildings,
			carla.MapLayer.Decals,
			carla.MapLayer.Foliage,
			carla.MapLayer.Ground,
			carla.MapLayer.ParkedVehicles,
			carla.MapLayer.Particles,
			carla.MapLayer.Props,
			carla.MapLayer.StreetLights,
			carla.MapLayer.Walls,
			carla.MapLayer.All
		]

	def restart(self):
		self.player_max_speed = 1.589
		self.player_max_speed_fast = 3.713
		# Keep same camera config if the camera manager exists.
		cam_index = self.camera_manager.index if self.camera_manager is not None else 0
		cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
		# Get a random blueprint.
		blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
		blueprint.set_attribute('role_name', self.actor_role_name)
		if blueprint.has_attribute('color'):
			color = random.choice(blueprint.get_attribute('color').recommended_values)
			blueprint.set_attribute('color', color)
		if blueprint.has_attribute('driver_id'):
			driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
			blueprint.set_attribute('driver_id', driver_id)
		if blueprint.has_attribute('is_invincible'):
			blueprint.set_attribute('is_invincible', 'true')
		# set the max speed
		if blueprint.has_attribute('speed'):
			self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
			self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
		else:
			print("No recommended values for 'speed' attribute")
		# Spawn the player.
		if self.player is not None:
			spawn_point = self.player.get_transform()
			spawn_point.location.z += 2.0
			spawn_point.rotation.roll = 0.0
			spawn_point.rotation.pitch = 0.0
			self.destroy()
			self.player = self.world.try_spawn_actor(blueprint, spawn_point)
			self.modify_vehicle_physics(self.player)
		while self.player is None:
			if not self.map.get_spawn_points():
				print('There are no spawn points available in your map/town.')
				print('Please add some Vehicle Spawn Point to your UE4 scene.')
				sys.exit(1)
			spawn_points = self.map.get_spawn_points()
			spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
			self.player = self.world.try_spawn_actor(blueprint, spawn_point)
			self.modify_vehicle_physics(self.player)
		# Set up the sensors.
		self.imu_sensor = IMUSensor(self.player, self.qForSendingImuDataToBalancerModule)
		self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
		self.camera_manager.transform_index = cam_pos_index
		self.camera_manager.set_sensor(cam_index, notify=False)
		actor_type = get_actor_display_name(self.player)

	def next_weather(self, reverse=False):
		self._weather_index += -1 if reverse else 1
		self._weather_index %= len(self._weather_presets)
		preset = self._weather_presets[self._weather_index]
		self.player.get_world().set_weather(preset[0])

	def next_map_layer(self, reverse=False):
		self.current_map_layer += -1 if reverse else 1
		self.current_map_layer %= len(self.map_layer_names)
		selected = self.map_layer_names[self.current_map_layer]

	def load_map_layer(self, unload=False):
		selected = self.map_layer_names[self.current_map_layer]
		if unload:
			self.world.unload_map_layer(selected)
		else:
			self.world.load_map_layer(selected)

	def modify_vehicle_physics(self, actor):
		#If actor is not a vehicle, we cannot use the physics control
		try:
			physics_control = actor.get_physics_control()
			physics_control.use_sweep_wheel_collision = True
			actor.apply_physics_control(physics_control)
		except Exception:
			pass

	def render(self, display):
		self.camera_manager.render(display)

	def destroy_sensors(self):
		self.camera_manager.sensor.destroy()
		self.camera_manager.sensor = None
		self.camera_manager.index = None

	def destroy(self):
		# if self.radar_sensor is not None:
		# 	self.toggle_radar()
		sensors = [
			self.camera_manager.sensor,
			self.imu_sensor.sensor]
		for sensor in sensors:
			if sensor is not None:
				sensor.stop()
				sensor.destroy()
		if self.player is not None:
			self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
	"""Class that handles keyboard input."""
	def __init__(self, world, start_in_autopilot):
		self._autopilot_enabled = start_in_autopilot
		if isinstance(world.player, carla.Vehicle):
			self._control = carla.VehicleControl()
			self._lights = carla.VehicleLightState.NONE
			world.player.set_autopilot(self._autopilot_enabled)
			world.player.set_light_state(self._lights)
		elif isinstance(world.player, carla.Walker):
			self._control = carla.WalkerControl()
			self._autopilot_enabled = False
			self._rotation = world.player.get_transform().rotation
		else:
			raise NotImplementedError("Actor type not supported")
		self._steer_cache = 0.0

	def parse_events(self, client, world, clock):
		if isinstance(self._control, carla.VehicleControl):
			current_lights = self._lights
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return True
			elif event.type == pygame.KEYUP:
				if self._is_quit_shortcut(event.key):
					return True
				elif event.key == K_BACKSPACE:
					if self._autopilot_enabled:
						world.player.set_autopilot(False)
						world.restart()
						world.player.set_autopilot(True)
					else:
						world.restart()
				elif event.key == K_F1:
					pass
				elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
					world.next_map_layer(reverse=True)
				elif event.key == K_v:
					world.next_map_layer()
				elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
					world.load_map_layer(unload=True)
				elif event.key == K_b:
					world.load_map_layer()
				elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
					pass
				elif event.key == K_TAB:
					world.camera_manager.toggle_camera()
				elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
					world.next_weather(reverse=True)
				elif event.key == K_c:
					world.next_weather()
				elif event.key == K_g:
					pass
				elif event.key == K_BACKQUOTE:
					world.camera_manager.next_sensor()
				elif event.key == K_n:
					world.camera_manager.next_sensor()
				elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
					if world.constant_velocity_enabled:
						world.player.disable_constant_velocity()
						world.constant_velocity_enabled = False
					else:
						world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
						world.constant_velocity_enabled = True
				elif event.key > K_0 and event.key <= K_9:
					world.camera_manager.set_sensor(event.key - 1 - K_0)
				elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
					world.camera_manager.toggle_recording()
				elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
					if (world.recording_enabled):
						client.stop_recorder()
						world.recording_enabled = False
					else:
						client.start_recorder("manual_recording.rec")
						world.recording_enabled = True
				elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
					# stop recorder
					client.stop_recorder()
					world.recording_enabled = False
					# work around to fix camera at start of replaying
					current_index = world.camera_manager.index
					world.destroy_sensors()
					# disable autopilot
					self._autopilot_enabled = False
					world.player.set_autopilot(self._autopilot_enabled)
					# replayer
					client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
					world.camera_manager.set_sensor(current_index)
				elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
					if pygame.key.get_mods() & KMOD_SHIFT:
						world.recording_start -= 10
					else:
						world.recording_start -= 1
				elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
					if pygame.key.get_mods() & KMOD_SHIFT:
						world.recording_start += 10
					else:
						world.recording_start += 1
				if isinstance(self._control, carla.VehicleControl):
					if event.key == K_q:
						self._control.gear = 1 if self._control.reverse else -1
					elif event.key == K_m:
						self._control.manual_gear_shift = not self._control.manual_gear_shift
						self._control.gear = world.player.get_control().gear
					elif self._control.manual_gear_shift and event.key == K_COMMA:
						self._control.gear = max(-1, self._control.gear - 1)
					elif self._control.manual_gear_shift and event.key == K_PERIOD:
						self._control.gear = self._control.gear + 1
					elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
						self._autopilot_enabled = not self._autopilot_enabled
						world.player.set_autopilot(self._autopilot_enabled)
					elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
						current_lights ^= carla.VehicleLightState.Special1
					elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
						current_lights ^= carla.VehicleLightState.HighBeam
					elif event.key == K_l:
						# Use 'L' key to switch between lights:
						# closed -> position -> low beam -> fog
						if not self._lights & carla.VehicleLightState.Position:
							pass
							current_lights |= carla.VehicleLightState.Position
							current_lights |= carla.VehicleLightState.LowBeam
						if self._lights & carla.VehicleLightState.LowBeam:
							current_lights |= carla.VehicleLightState.Fog
						if self._lights & carla.VehicleLightState.Fog:
							current_lights ^= carla.VehicleLightState.Position
							current_lights ^= carla.VehicleLightState.LowBeam
							current_lights ^= carla.VehicleLightState.Fog
					elif event.key == K_i:
						current_lights ^= carla.VehicleLightState.Interior
					elif event.key == K_z:
						current_lights ^= carla.VehicleLightState.LeftBlinker
					elif event.key == K_x:
						current_lights ^= carla.VehicleLightState.RightBlinker

		if not self._autopilot_enabled:
			if isinstance(self._control, carla.VehicleControl):
				self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
				self._control.reverse = self._control.gear < 0
				# Set automatic control-related vehicle lights
				if self._control.brake:
					current_lights |= carla.VehicleLightState.Brake
				else: # Remove the Brake flag
					current_lights &= ~carla.VehicleLightState.Brake
				if self._control.reverse:
					current_lights |= carla.VehicleLightState.Reverse
				else: # Remove the Reverse flag
					current_lights &= ~carla.VehicleLightState.Reverse
				if current_lights != self._lights: # Change the light state only if necessary
					self._lights = current_lights
					world.player.set_light_state(carla.VehicleLightState(self._lights))
			elif isinstance(self._control, carla.WalkerControl):
				self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
			# world.player.apply_control(self._control) # vladddd46

	def _parse_vehicle_keys(self, keys, milliseconds):
		if keys[K_UP] or keys[K_w]:
			self._control.throttle = min(self._control.throttle + 0.01, 1)
		else:
			self._control.throttle = 0.0

		if keys[K_DOWN] or keys[K_s]:
			self._control.brake = min(self._control.brake + 0.2, 1)
		else:
			self._control.brake = 0

		steer_increment = 5e-4 * milliseconds
		if keys[K_LEFT] or keys[K_a]:
			if self._steer_cache > 0:
				self._steer_cache = 0
			else:
				self._steer_cache -= steer_increment
		elif keys[K_RIGHT] or keys[K_d]:
			if self._steer_cache < 0:
				self._steer_cache = 0
			else:
				self._steer_cache += steer_increment
		else:
			self._steer_cache = 0.0
		self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
		self._control.steer = round(self._steer_cache, 1)
		self._control.hand_brake = keys[K_SPACE]

	def _parse_walker_keys(self, keys, milliseconds, world):
		self._control.speed = 0.0
		if keys[K_DOWN] or keys[K_s]:
			self._control.speed = 0.0
		if keys[K_LEFT] or keys[K_a]:
			self._control.speed = .01
			self._rotation.yaw -= 0.08 * milliseconds
		if keys[K_RIGHT] or keys[K_d]:
			self._control.speed = .01
			self._rotation.yaw += 0.08 * milliseconds
		if keys[K_UP] or keys[K_w]:
			self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
		self._control.jump = keys[K_SPACE]
		self._rotation.yaw = round(self._rotation.yaw, 1)
		self._control.direction = self._rotation.get_forward_vector()

	@staticmethod
	def _is_quit_shortcut(key):
		return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


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



# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================

class IMUSensor(object):
	def __init__(self, parent_actor, qForSendingImuDataToBalancerModule):
		self.sensor = None
		self._parent = parent_actor
		self.accelerometer = (0.0, 0.0, 0.0)
		self.gyroscope = (0.0, 0.0, 0.0)
		self.compass = 0.0
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.imu')
		bp.set_attribute('sensor_tick', "0.1")
		self.sensor = world.spawn_actor(
			bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(
			lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data, qForSendingImuDataToBalancerModule))

	@staticmethod
	def _IMU_callback(weak_self, sensor_data, qForSendingImuDataToBalancerModule):
		self = weak_self()
		if not self:
			return
		limits = (-99.9, 99.9)
		self.accelerometer = (
			max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
			max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
			max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
		self.gyroscope = (
			max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
			max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
			max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
		self.compass = math.degrees(sensor_data.compass)

		velocity = self._parent.get_velocity()
		accelerometerData = sensor_data.accelerometer
		gyroscopeData     = sensor_data.gyroscope
		sendData = {"accel": {"x": round(accelerometerData.x, 4), 
							  "y": round(accelerometerData.y, 4), 
							  "z": round(accelerometerData.z, 4)}, 
					"gyro":  {"x": round(gyroscopeData.x, 4), 
							  "y": round(gyroscopeData.y, 4), 
							  "z": round(gyroscopeData.z, 4)},
					"velocity": {"x":  round(velocity.x, 4),
								 "y":  round(velocity.y, 4),
								 "z":  round(velocity.z, 4)}
					}
		print(sendData)
		sendDataJson = json.dumps(sendData)
		qForSendingImuDataToBalancerModule.put(sendDataJson)


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
		bound_x = 0.5 + self._parent.bounding_box.extent.x
		bound_y = 0.5 + self._parent.bounding_box.extent.y
		bound_z = 0.5 + self._parent.bounding_box.extent.z
		Attachment = carla.AttachmentType

		if not self._parent.type_id.startswith("walker.pedestrian"):
			self._camera_transforms = [
				(carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
				(carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
		else:
			self._camera_transforms = [
				(carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
				(carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
				(carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

		self.transform_index = 1
		self.sensors = [
			['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
			['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
			['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
			['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
			['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
			['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
				'Camera Semantic Segmentation (CityScapes Palette)', {}],
			['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
			['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
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
			elif item[0].startswith('sensor.lidar'):
				self.lidar_range = 50

				for attr_name, attr_value in item[3].items():
					bp.set_attribute(attr_name, attr_value)
					if attr_name == 'range':
						self.lidar_range = float(attr_value)

			item.append(bp)
		self.index = None

	def toggle_camera(self):
		self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
		self.set_sensor(self.index, notify=False, force_respawn=True)

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
				self._camera_transforms[self.transform_index][0],
				attach_to=self._parent,
				attachment_type=self._camera_transforms[self.transform_index][1])
			# We need to pass the lambda a weak reference to self to avoid
			# circular reference.
			weak_self = weakref.ref(self)
			self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
		self.index = index

	def next_sensor(self):
		self.set_sensor(self.index + 1)

	def toggle_recording(self):
		self.recording = not self.recording

	def render(self, display):
		if self.surface is not None:
			display.blit(self.surface, (0, 0))

	@staticmethod
	def _parse_image(weak_self, image):
		self = weak_self()
		if not self:
			return
		if self.sensors[self.index][0].startswith('sensor.lidar'):
			points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
			points = np.reshape(points, (int(points.shape[0] / 4), 4))
			lidar_data = np.array(points[:, :2])
			lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
			lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
			lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
			lidar_data = lidar_data.astype(np.int32)
			lidar_data = np.reshape(lidar_data, (-1, 2))
			lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
			lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
			lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
			self.surface = pygame.surfarray.make_surface(lidar_img)
		elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
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
		if self.recording:
			image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
	pygame.init()
	pygame.font.init()
	world = None

	try:
		client = carla.Client(args.carla_ip, args.carla_port)
		client.set_timeout(5.0)

		display = pygame.display.set_mode(
			(args.width, args.height),
			pygame.HWSURFACE | pygame.DOUBLEBUF)
		display.fill((0,0,0))
		pygame.display.flip()

		hud = HUD(args.width, args.height)
		qForSendingImuDataToBalancerModule = Queue()
		world = World(client.get_world(), hud, args, qForSendingImuDataToBalancerModule)
		controller = KeyboardControl(world, args.autopilot)
#
		sendThread = threading.Thread(target=sendImuDataToRemoteServer_THREAD, args=(qForSendingImuDataToBalancerModule, args))
		sendThread.start()

		receivedThread = threading.Thread(target=ReceiveSteerWheelAngleFromRemoteServer_THREAD, args=(args, world.player))
		receivedThread.start()

		motorbike = world.player
		clock = pygame.time.Clock()
		while True:
			clock.tick_busy_loop(60)
			if controller.parse_events(client, world, clock):
				return
			world.render(display)
			pygame.display.flip()
	finally:
		if (world and world.recording_enabled):
			client.stop_recorder()

		if world is not None:
			world.destroy()

		pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
def main():
	args = parseArguments()

	args.width, args.height = [int(x) for x in args.res.split('x')]
	log_level = logging.DEBUG if args.debug else logging.INFO
	logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
	logging.info('listening to server %s:%s', args.carla_ip, args.carla_port)
	try:
		game_loop(args)
	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')

if __name__ == '__main__':
	main()
