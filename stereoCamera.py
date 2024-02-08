import carla
import numpy as np
import sys
import os
sys.path.append(f'{os.getcwd()}/..') 
from sensorsP import StereoCameraP 

class StereoCamera:
    # documentation can be found at:
    # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
    #settings posizione camere [m]
    
    def __init__(self, world, ego, mode="rgba"):
        
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(StereoCameraP.image_size_x))
        camera_bp.set_attribute('image_size_y', str(StereoCameraP.image_size_y)) 
        camera_bp.set_attribute('fov', str(StereoCameraP.fov))
        camera_bp.set_attribute('fstop', str(StereoCameraP.fstop))
        camera_bp.set_attribute('iso', str(StereoCameraP.iso))
        camera_bp.set_attribute('shutter_speed', str(StereoCameraP.shutter_speed))
        camera_bp.set_attribute('lens_flare_intensity', str(StereoCameraP.lens_flare_intensity))
        camera_bp.set_attribute('bloom_intensity', str(StereoCameraP.bloom_intensity))
        camera_bp.set_attribute('gamma', str(StereoCameraP.gamma))
        camera_bp.set_attribute('sensor_tick', str(StereoCameraP.sensor_tick))

        # set the init transform for the stereocamera based on static members
        camera1_init_trans = carla.Transform(carla.Location(z=StereoCameraP.camera_height,
                                                            x=StereoCameraP.camera_displacement, 
                                                            y=-StereoCameraP.relative_distance/2))

        camera2_init_trans = carla.Transform(carla.Location(z=StereoCameraP.camera_height,
                                                            x=StereoCameraP.camera_displacement, 
                                                            y=StereoCameraP.relative_distance/2))

        self.camera1 = world.spawn_actor(camera_bp,camera1_init_trans,attach_to=ego)
        self.camera2 = world.spawn_actor(camera_bp,camera2_init_trans,attach_to=ego) 
        if mode=="rgba":
            self._slice = None
        elif mode=="rgb":
            self._slice = 3 
        else:
            raise Exception("modes allowed for stereocamera are rgb and rgba")

        self.camera1_data = np.zeros((StereoCameraP.image_size_y, StereoCameraP.image_size_x,4))
        self.camera2_data = np.zeros((StereoCameraP.image_size_y, StereoCameraP.image_size_x,4))

    def listen(self):
        # meglio due funzioni separate che fare un if ogni volta
        def camera1_callback(image):
            self.camera1_data = np.reshape(np.copy(image.raw_data),(StereoCameraP.image_size_y,StereoCameraP.image_size_x,4))
        def camera2_callback(image):
            self.camera2_data = np.reshape(np.copy(image.raw_data),(StereoCameraP.image_size_y,StereoCameraP.image_size_x,4))

        self.camera1.listen(lambda image: camera1_callback(image))
        self.camera2.listen(lambda image: camera2_callback(image))

    def update(self):
        return np.hstack((self.camera1_data[:,:,:self._slice], self.camera2_data[:,:,:self._slice]))

    def getCamera1(self):
        return self.camera1_data[:,:,:self._slice]
    
    def getCamera2(self):
        return self.camera2_data[:,:,:self._slice]