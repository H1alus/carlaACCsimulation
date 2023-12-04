import carla
import cv2
import numpy as np

class StereoCamera:
    image_w = 640
    image_h = 360
    
    def __init__(self, world, ego, camera_height=1.5, camera_displacement=0.4, relative_distance=0.2):

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(StereoCamera.image_w))
        camera_bp.set_attribute('image_size_y', str(StereoCamera.image_h)) 
        # set the init transform for the stereocamera based on static members
        camera1_init_trans = carla.Transform(carla.Location(z=camera_height,x=camera_displacement, y=-relative_distance/2))
        camera2_init_trans = carla.Transform(carla.Location(z=camera_height,x=camera_displacement, y=relative_distance/2))
        #this creates the camera in the sim
        self.camera1 = world.spawn_actor(camera_bp,camera1_init_trans,attach_to=ego)
        self.camera2 = world.spawn_actor(camera_bp,camera2_init_trans,attach_to=ego) 

        self.camera1_data = np.zeros((StereoCamera.image_h, StereoCamera.image_w,4))
        self.camera2_data = np.zeros((StereoCamera.image_h, StereoCamera.image_w,4))

    def listen(self):
        # sicuramente deve diventare un'unica funzione
        def camera1_callback(image):
            self.camera1_data = np.reshape(np.copy(image.raw_data),(StereoCamera.image_h,StereoCamera.image_w,4))
        def camera2_callback(image):
            self.camera2_data = np.reshape(np.copy(image.raw_data),(StereoCamera.image_h,StereoCamera.image_w,4))

        # this actually opens a live stream from the camera
        self.camera1.listen(lambda image: camera1_callback(image))
        self.camera2.listen(lambda image: camera2_callback(image))
        cv2.namedWindow('control view',cv2.WINDOW_AUTOSIZE)

    def update(self):
        camera_data = np.hstack((self.camera1_data, self.camera2_data))
        cv2.imshow('control view', camera_data)