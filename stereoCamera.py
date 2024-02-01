import carla
import numpy as np

class StereoCamera:
    # documentation can be found at:
    # https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera
    #settings posizione camere [m]
    camera_height=1.5
    camera_displacement=0.4
    relative_distance=0.2
    
    # settings lente
    image_size_x = 640
    image_size_y = 360
    fov = 90
    fstop = 1.4
    iso = 100
    shutter_speed = 200
    lens_flare_intensity = 0.1
    
    # attributi di post processing
    bloom_intensity = 0.675
    gamma = 2.2
    sensor_tick = 0
    
    def __init__(self, world, ego):

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(StereoCamera.image_size_x))
        camera_bp.set_attribute('image_size_y', str(StereoCamera.image_size_y)) 
        camera_bp.set_attribute('fov', str(StereoCamera.fov))
        camera_bp.set_attribute('fstop', str(StereoCamera.fstop))
        camera_bp.set_attribute('iso', str(StereoCamera.iso))
        camera_bp.set_attribute('shutter_speed', str(StereoCamera.shutter_speed))
        camera_bp.set_attribute('lens_flare_intensity', str(StereoCamera.lens_flare_intensity))
        camera_bp.set_attribute('bloom_intensity', str(StereoCamera.bloom_intensity))
        camera_bp.set_attribute('gamma', str(StereoCamera.gamma))
        camera_bp.set_attribute('sensor_tick', str(StereoCamera.sensor_tick))

        # set the init transform for the stereocamera based on static members
        camera1_init_trans = carla.Transform(carla.Location(z=StereoCamera.camera_height,
                                                            x=StereoCamera.camera_displacement, 
                                                            y=-StereoCamera.relative_distance/2))

        camera2_init_trans = carla.Transform(carla.Location(z=StereoCamera.camera_height,
                                                            x=StereoCamera.camera_displacement, 
                                                            y=StereoCamera.relative_distance/2))

        self.camera1 = world.spawn_actor(camera_bp,camera1_init_trans,attach_to=ego)
        self.camera2 = world.spawn_actor(camera_bp,camera2_init_trans,attach_to=ego) 

        self.camera1_data = np.zeros((StereoCamera.image_size_y, StereoCamera.image_size_x,4))
        self.camera2_data = np.zeros((StereoCamera.image_size_y, StereoCamera.image_size_x,4))

    def listen(self):
        # meglio due funzioni separate che fare un if ogni volta
        def camera1_callback(image):
            self.camera1_data = np.reshape(np.copy(image.raw_data),(StereoCamera.image_size_y,StereoCamera.image_size_x,4))
        def camera2_callback(image):
            self.camera2_data = np.reshape(np.copy(image.raw_data),(StereoCamera.image_size_y,StereoCamera.image_size_x,4))

        self.camera1.listen(lambda image: camera1_callback(image))
        self.camera2.listen(lambda image: camera2_callback(image))

    def update(self):
        return np.hstack((self.camera1_data, self.camera2_data))

    def getCamera1(self):
        return self.camera1_data