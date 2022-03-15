import cv2
import copy

import numpy as np


def pos2pixel(pos):
    pixel = [0, 0]
    pixel[0] = int(pos[0] * 9) + 400   # Left: - || Right: +
    pixel[1] = int(pos[1] * 9) - 800   # Up: - || Down: +
    return pixel


def draw_waypoints(image, points):
    img_with_waypoints = image
    point_color = (255, 0, 0)
    for point in points:
        pixel = pos2pixel(point)
        print(pixel)
        img_with_waypoints = cv2.circle(img_with_waypoints, pixel, 8, point_color, 4)
    return img_with_waypoints


img_init = cv2.imread('maps/Town02_resize.png')
print("RAW IMAGE: ", img_init.shape)
img_size = img_init.shape[:2]

vehicle_info = np.load("vehicle_info.npy")
print(vehicle_info.shape)
vehicle_pos = vehicle_info[:, :2]
print(vehicle_pos.shape)

img = copy.deepcopy(img_init)
img = draw_waypoints(img, vehicle_pos)
cv2.imwrite("img_with_track.png", img)
