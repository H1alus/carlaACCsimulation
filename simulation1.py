import carla
import numpy as np
import cv2
import sys
import os
import random
sys.path.append(f'{os.getcwd()}/simulator/PythonAPI/carla') 
from agents.navigation.global_route_planner import GlobalRoutePlanner
import matplotlib.pyplot as plt
from tracker import Tracker


###############################################################################
#context
###############################################################################
client = carla.Client('host.docker.internal', 2000)
client.load_world('Town04_Opt')
world = client.get_world()
world.unload_map_layer(carla.MapLayer.Buildings)
world.unload_map_layer(carla.MapLayer.ParkedVehicles)
world.unload_map_layer(carla.MapLayer.Decals)
world.unload_map_layer(carla.MapLayer.Foliage)
world.unload_map_layer(carla.MapLayer.Props)

world.set_weather(carla.WeatherParameters.ClearNoon)

pos = carla.Transform(carla.Location(x=402.784515, y=-149.542175, z=0.281942), 
                      carla.Rotation(pitch=0.000000, yaw=-89.401421, roll=0.000000))
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
from stereoCamera import StereoCamera
stereocam = StereoCamera(world, ego)
stereocam.listen()
###############################################################################
#route planning
###############################################################################
sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
first_half = grp.trace_route(pos.location, end_pos.location)
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
second_half = grp.trace_route(end_pos.location, pos.location)[:-1]
route = first_half + second_half
data = ([x[0].transform.location.x for x in route], [y[0].transform.location.y for y in route])
# for waypoint in route:
#     world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
#         color=carla.Color(r=0, g=0, b=255), life_time=600.0,
#         persistent_lines=True)
###############################################################################
#main loop
###############################################################################
def run():
    tracker = Tracker(ego, route)
    throttle,_ = tracker.desired_speed(10)
    ego.apply_control(carla.VehicleControl(throttle=throttle, steer=0))
    while True:
        # Carla Tick
        world.tick()
        # exit by pressing q
        if cv2.waitKey(1) == ord('q'):
            break
        v = ego.get_velocity()
        v = np.array([v.x, v.y, v.z])
        v = np.sqrt(v.dot(v))*3.6
        angle = tracker.keepTrack()
        throttle, brake = tracker.desired_speed(130)
        """image = cv2.putText(
                            image, 'Speed: ' + str(int(np.ceil(v))) + ' Km/h', (30, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 
                            1, cv2.LINE_AA
                            )"""
        stereocam.update()
        ego.apply_control(carla.VehicleControl(throttle=throttle, steer=angle, brake=brake))

if __name__ == "__main__":
    run()