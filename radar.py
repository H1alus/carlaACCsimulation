import carla
import numpy as np
import math
import sys
import os
sys.path.append(f'{os.getcwd()}/..') 
from sensorsP import RadarP

class Radar:
    # documentation can be found at:
    # https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor

    def __init__(self, world, ego):
        self.world = world
        radar_bp = world.get_blueprint_library().find("sensor.other.radar")
        radar_bp.set_attribute('horizontal_fov', str(RadarP.HORIZONTAL_FOV))
        radar_bp.set_attribute('vertical_fov', str(RadarP.VERTICAL_FOV))
        radar_bp.set_attribute('range', str(RadarP.RANGE)) 
        radar_bp.set_attribute("points_per_second", str(RadarP.POINTS_PER_SECOND))
        radar_bp.set_attribute("sensor_tick", str(RadarP.SENSOR_TICK))
        radar_init_trans = carla.Transform(carla.Location(x=RadarP.RADAR_FORWARD_DISPLACEMENT, 
                                                          z=RadarP.RADAR_HEIGHT), 
                                                          carla.Rotation(pitch=RadarP.RADAR_PITCH))
        self._radar = world.spawn_actor(radar_bp, radar_init_trans, attach_to=ego,attachment_type = carla.AttachmentType.Rigid )
        self._radar_data = None
    
    def listen_debug(self):
        def rad_callback(radar_data):
            self._radar_data =  np.zeros((len(radar_data),),dtype='f,f,f,f')
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            for i, detect in enumerate(radar_data):
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

                norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                self.world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.05,
                    life_time=0.02,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
                self._radar_data[i] = (detect.depth, detect.azimuth, detect.altitude, detect.velocity)
        self._radar.listen(lambda radar_data: rad_callback(radar_data))

    def listen_debug_ground_remove(self):
        def rad_callback(radar_data):
            self._radar_data =  np.zeros((len(radar_data),),dtype='f,f,f,f')
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            for i, detect in enumerate(radar_data):
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)
                fw_vec = carla.Vector3D(x=detect.depth)
                carla.Transform(
                    carla.Location(),
                    carla.Rotation(
                        pitch=current_rot.pitch + alt,
                        yaw=current_rot.yaw + azi,
                        roll=current_rot.roll)).transform(fw_vec)

                dist  = detect.depth  # distanza
                alpha = -detect.altitude  # altitude (angolo rispetto al terreno)
                arc_sin_value = math.asin(1 / dist)
                min_value = arc_sin_value - (arc_sin_value * RadarP.GROUND_CORRECT_PERCENTAGE/100)
                max_value = arc_sin_value + (arc_sin_value * RadarP.GROUND_CORRECT_PERCENTAGE/100)
                if not (min_value <= alpha <= max_value):
                    r = 0
                    g=255       # non è terreno
                    b = 0
                else: 
                    g = 0
                    r=255       # è il terreno
                    b=0
                
                self.world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.05,
                    life_time=0.02,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
                self._radar_data[i] = (detect.depth, detect.azimuth, detect.altitude, detect.velocity)
        self._radar.listen(lambda radar_data: rad_callback(radar_data))

    def listen(self):
        def rad_callback(radar_data):
            self._radar_data =  np.zeros((len(radar_data),),dtype='f,f,f,f')
            #velocity_range = 7.5 # m/s
            #current_rot = radar_data.transform.rotation
            for i, detect in enumerate(radar_data):
                #azi = math.degrees(detect.azimuth)
                #alt = math.degrees(detect.altitude)
                #norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                
                self._radar_data[i] = (detect.depth, detect.azimuth, detect.altitude, detect.velocity)
        self._radar.listen(lambda radar_data: rad_callback(radar_data))
    
    def update(self):
        return self._radar_data