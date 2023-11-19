import carla
import numpy as np
import cv2
import time
import random
from simple_pid import PID
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import sys
import os
sys.path.append(f'{os.getcwd()}/simulator/PythonAPI/carla') 
from agents.navigation.global_route_planner import GlobalRoutePlanner
from tracker import Tracker

#adding params to display text to image
font = cv2.FONT_HERSHEY_SIMPLEX
# org - defining lines to display telemetry values on the screen
org = (30, 30) # this line will be used to show current speed
org2 = (30, 50) # this line will be used for future steering angle
org3 = (30, 70) # and another line for future telemetry outputs
org4 = (30, 90) # and another line for future telemetry outputs
org3 = (30, 110) # and another line for future telemetry outputs
fontScale = 0.5
# white color
color = (255, 255, 255)
# Line thickness of 2 px
thickness = 1

def update_line(data):
    line.set_data(data[0], data[1])
    return line,

def desired_speed(vehicle, speed):
    """
    convert speed (Km/h) to the throttle parameter
    based on current speed
    
    ## parameteres:
    vehicle: carla.Vehicle object, the vehicle to apply the 

    speed: target speed we want to obtain

    ## returns:
    throttle: float - the required throttle to achieve the required speed
    """
    v = vehicle.get_velocity()
    v = np.array([v.x, v.y, v.z])
    v = np.sqrt(v.dot(v))
    speed = speed/3.6
    threshold = 1
    if v <= speed - threshold:
        return (speed - v + 1) % 2 + 1
    elif v > speed:
        return 0
    else:
        return 0.4


###############################################################################
#context
###############################################################################
client = carla.Client('host.docker.internal', 2000)
client.load_world('Town04_Opt')
world = client.get_world()
world.unload_map_layer(carla.MapLayer.Buildings)
world.unload_map_layer(carla.MapLayer.ParkedVehicles)
#world.unload_map_layer(carla.MapLayer.Decals)
world.unload_map_layer(carla.MapLayer.Foliage)

pos = carla.Transform(carla.Location(x=402.784515, y=-149.542175, z=0.281942), 
                      carla.Rotation(pitch=0.000000, yaw=-89.401421, roll=0.000000))
# end_pos = carla.Transform(carla.Location(x=-4.865314, y=131.391022, z=1.029803), 
#                       carla.Rotation(pitch=-25.136202, yaw=88.529762, r
#adding params to display text to image
font = cv2.FONT_HERSHEY_SIMPLEX
# org - defining lines to display telemetry values on the screen
org = (30, 30) # this line will be used to show current speed
org2 = (30, 50) # this line will be used for future steering angle
org3 = (30, 70) # and another line for future telemetry outputs
org4 = (30, 90) # and another line for future telemetry outputs
org3 = (30, 110) # and another line for future telemetry outputsoll=0.000025))
end_pos = carla.Transform(carla.Location(x=-391.342010, y=26.208113, z=1.766472), 
                          carla.Rotation(pitch=-7.819187, yaw=0.168475, roll=0.000083))
spawn_points = world.get_map().get_spawn_points()
###############################################################################
#ego vehicle
###############################################################################
ego_spawn = pos
#ego_spawn.location -= 10*(pos.get_forward_vector())
ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
ego_bp.set_attribute('role_name', 'hero')
ego = world.spawn_actor(ego_bp, ego_spawn)
###############################################################################
#visualization camera
###############################################################################
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640')
camera_bp.set_attribute('image_size_y', '360')
camera_init_trans = carla.Transform(carla.Location(z=3,x=-10))
#this creates the camera in the sim
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=ego)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# this actually opens a live stream from the camera
camera.listen(lambda image: camera_callback(image,camera_data))
cv2.namedWindow('control view',cv2.WINDOW_AUTOSIZE)
cv2.imshow('control view',camera_data['image'])
###############################################################################
#route planning
###############################################################################
sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
first_half = grp.trace_route(pos.location, end_pos.location)
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
second_half = grp.trace_route(end_pos.location, pos.location)[:-1]
route = first_half + second_half
#draw the route in sim window - Note it does not get into the camera of the car
for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=600.0,
        persistent_lines=True)
###############################################################################
#main loop
###############################################################################
tracker = Tracker(ego, route)
ego.apply_control(carla.VehicleControl(throttle=0.2, steer=0))
spectator = world.get_spectator()
data = ([],[])
while True:
    start = time.time()
    # Carla Tick
    world.tick()
    # exit by pressing q
    if cv2.waitKey(1) == ord('q'):
        quit = True
        break
    # egoTransform = ego.get_transform()
    # egoLocation = egoTransform.location
    # data[0].append(egoLocation.x)
    # data[1].append(egoLocation.y)
    image = camera_data['image']
    v = ego.get_velocity()
    v = np.array([v.x, v.y, v.z])
    v = np.sqrt(v.dot(v))*3.6
    image = cv2.putText(
                        image, 'Speed: ' + str(int(np.ceil(v))) + ' Km/h', (30, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale, color, 
                        thickness, cv2.LINE_AA
                        )
    speeds = [50, 70, 80, 90]
    ego.apply_control(carla.VehicleControl(throttle=desired_speed(ego, 50), steer=tracker.keepTrack()))
    cv2.imshow('control view',image)
    end = time.time()
    time.sleep(max(0.05 - (end - start), 0))