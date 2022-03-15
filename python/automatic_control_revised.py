# -*- coding: utf-8 -*-

"""Revised automatic control
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import sys
import time
import random

import carla
from queue import Queue
from queue import Empty

import numpy as np

from carla_utils import get_vehicle_info, get_transform_location
from python.agents.navigation.behavior_agent import BehaviorAgent


def sensor_callback(sensor_data, sensor_queue, sensor_name, vehicle):
    data_dir = 'D:\\DataSet\\trans\\demo\\%06d' % sensor_data.frame
    if not os.path.exists(data_dir):
        os.mkdir(data_dir)

    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join(data_dir, 'raw_lidar.ply'))
        np.save(os.path.join(data_dir, 'vehicle_info.npy'), np.array(get_vehicle_info(vehicle)))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join(data_dir, 'raw_image.png'))
    sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    sensor_list = []
    vehicle_info_list = []
    destination_list = [14, 67, 4]
    destination_list = [4]
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        world = client.load_world('Town02')

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # create sensor queue
        sensor_queue = Queue()

        blueprint_library = world.get_blueprint_library()

        # read all valid spawn points
        all_default_spawn = world.get_map().get_spawn_points()

        map_spawn_points = []
        # print(all_default_spawn)
        for default_spawn in all_default_spawn:
            map_spawn_points.append(get_transform_location(default_spawn))
        np.save("map2_spawn_points.npy", np.array(map_spawn_points))

        # randomly choose one as the start point
        spawn_point = all_default_spawn[67]
        destination = all_default_spawn[destination_list[0]]
        destination_list.pop(0)

        # create the blueprint library
        ego_vehicle_bp = blueprint_library.find('vehicle.dodge.charger_police_2020')
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # create directory for outputs
        output_path = '../outputs/output_synchronized'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # add a camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        # camera relative position related to the vehicle
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        # set the callback function
        camera.listen(lambda image: sensor_callback(image, sensor_queue, "camera", vehicle))
        sensor_list.append(camera)

        # we also add a lidar on it
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))

        # set the relative location
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        # spawn the lidar
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        lidar.listen(
            lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar", vehicle))
        sensor_list.append(lidar)

        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        agent = BehaviorAgent(vehicle, behavior='normal')

        # # set the destination spot
        # spawn_points = world.get_map().get_spawn_points()
        # random.shuffle(spawn_points)
        #
        # # to avoid the destination and start position same
        # if spawn_points[0].location != agent.vehicle.get_location():
        #     destination = spawn_points[0]
        # else:
        #     destination = spawn_points[1]

        # generate the route
        agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)

        while True:
            # print(time.time())
            agent.update_information(vehicle)

            world.tick()
            
            if len(agent._local_planner.waypoints_queue) < 1:
                if len(destination_list) == 0:
                    print('======== Success, Arrivied at Target Point!')
                    break
                destination = all_default_spawn[destination_list[0]]
                destination_list.pop(0)
                agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)
                agent.update_information(vehicle)
                
            # top view
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))

            speed_limit = vehicle.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step(debug=True)
            vehicle.apply_control(control)

            vehicle_info_list.append(get_vehicle_info(vehicle))

            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    print("Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

            except Empty:
                print("Some of the sensor information is missed")

    finally:
        np.save("vehicle_info.npy", np.array(vehicle_info_list))
        world.apply_settings(origin_settings)
        vehicle.destroy()
        for sensor in sensor_list:
            sensor.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
