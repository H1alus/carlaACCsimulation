import carla
import numpy as np
import random
import time
import cv2
from simple_pid import PID
import sys
import os
sys.path.append(f'{os.getcwd()}/simulator/PythonAPI/carla') 
from agents.navigation.global_route_planner import GlobalRoutePlanner
WHEELBASE = 2.847


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

class AutoSteer:
    def __init__(self, ego, route):
        self.ego = ego
        self.route = route
        self.prev_out = 0
        self.pid = PID(1.2, 2, 4)
        self.idx = 0
        self.wps = []
        self.wps = np.array([wp[0].transform.location for wp in self.route])

    def lookAhead(self):
        carTransform = self.ego.get_transform()
        carLocation = carTransform.location
        carnorm = carTransform.get_forward_vector()
        carnorm = np.array([carnorm.x, carnorm.y])

        for wp in self.wps:
            dist = np.array([wp.x - carLocation.x, wp.y - carLocation.y])
            distnorm = dist / np.array(np.sqrt(dist.dot(dist)))
            if distnorm.dot(carnorm) >= 0:
                idx = np.where(self.wps == wp)[0][0]
                break
        return self.wps[idx:idx+4]

    def delta_id(self, wp):
        carTransform = self.ego.get_transform()
        carVector = carTransform.location
        carVector = np.array([carVector.x, carVector.y])
        wp = wp[0]
        wp = np.array([wp.x, wp.y])
        v = np.array([wp[0] - carVector[0], wp[1] - carVector[1]])
        ld = np.sqrt(v.dot(v))
        vnorm = np.array([v[0]/ld, v[1]/ld])
        
        carnorm = carTransform.get_forward_vector()
        carnorm = np.array([carnorm.x, carnorm.y])
        alpha = np.arccos(carnorm.dot(vnorm))
        angle = -1*np.arctan(2*WHEELBASE*np.sin(alpha)/ld)
        # if np.abs(angle) > np.pi/4:
        #     angle = np.sign(angle)*np.pi/4
        return angle

    #equals to deltaT
    def angle(self):
        wpAhead = self.lookAhead()
        self.idx += 1
        Delta_id = self.delta_id(wpAhead[-1:])
        error = Delta_id - self.prev_out
        out = self.pid(error)
        self.prev_out = out
        return out, wpAhead[-1:][0]


###############################################################################
#context
###############################################################################
client = carla.Client('host.docker.internal', 2000)
world = client.get_world()
client.load_world('Town04')
pos = carla.Transform(carla.Location(x=402.784515, y=-149.542175, z=0.281942), 
                      carla.Rotation(pitch=0.000000, yaw=-89.401421, roll=0.000000))
end_pos = carla.Transform(carla.Location(x=-4.865314, y=131.391022, z=1.029803), 
                      carla.Rotation(pitch=-25.136202, yaw=88.529762, roll=0.000025))
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
cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera',camera_data['image'])
###############################################################################
#route planning
###############################################################################
sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
route = grp.trace_route(pos.location, end_pos.location)
#draw the route in sim window - Note it does not get into the camera of the car
for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=600.0,
        persistent_lines=True)
###############################################################################
#main loop
###############################################################################
autosteer = AutoSteer(ego, route)
ego.apply_ackermann_control(carla.VehicleAckermannControl(speed=10, acceleration=1000, 
                                               steer=0))
for wp_i in range(len(route)):
    # Carla Tick
    world.tick()
    if cv2.waitKey(1) == ord('q'):
        quit = True
        break
    image = camera_data['image']
    angle, curr_wp = autosteer.angle()
    image = cv2.putText(image, 'Next waypoint: '+str(curr_wp), org2, font, fontScale, color, thickness, cv2.LINE_AA)
    ego.apply_ackermann_control(carla.VehicleAckermannControl(speed=30, acceleration=100, 
                                               steer=angle))
    cv2.imshow('RGB Camera',image)
