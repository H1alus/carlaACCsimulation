import carla
import numpy as np
import math

class Radar:
    POINTS = 1500
    def __init__(self, world, ego):
        self.world = world
        radar_bp = world.get_blueprint_library().find("sensor.other.radar")
        radar_init_trans = carla.Transform(carla.Location(x=2.8, z=1.0), carla.Rotation(pitch=5))
        self._radar = world.spawn_actor(radar_bp, radar_init_trans, attach_to=ego,attachment_type = carla.AttachmentType.Rigid )
        self._radar_data = np.zeros((Radar.POINTS,),dtype='f,f,f,f')
    
    def listen(self):
        obj_data = self._radar_data
        def rad_callback(radar_data):
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
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
                obj_data[i] = (detect.depth, detect.azimuth, detect.altitude, norm_velocity*3.6)
        self._radar.listen(lambda radar_data: rad_callback(radar_data))
        

    
    def update(self):
        return self._radar_data