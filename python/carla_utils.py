import numpy as np


def get_vehicle_info(vehicle):
    angular_velocity = vehicle.get_angular_velocity()
    acceleration = vehicle.get_acceleration()
    velocity = vehicle.get_velocity()
    transform = vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation
    vehicle_info = np.array([
        location.x,
        location.y,
        location.z,
        rotation.pitch,
        rotation.roll,
        rotation.yaw,
        velocity.x,
        velocity.y,
        velocity.z,
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
        acceleration.x,
        acceleration.y,
        acceleration.z,
    ])
    return vehicle_info
