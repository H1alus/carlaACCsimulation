import carla
import time
from threading import Thread, Event
#import cv2
import random
import numpy as np
import math

class Context:
    def __init__(self, simple=True):
        if type(simple) != bool:
            raise TypeError('simplify must be of type Bool')
        
        self.client = carla.Client('host.docker.internal', 2000)
        self.world = self.client.get_world()
        self.client.load_world('Town04')
        self.settings = self.world.get_settings()
        self.settings.synchronous_mode = True # Enables synchronous mode
        self.settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(self.settings)

        if simple:
            self.simplify()

    def simplify(self):
        self.world.unload_map_layer(carla.MapLayer.Buildings)
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        self.world.unload_map_layer(carla.MapLayer.Foliage)

# testbench class
class Simulation:
    def __init__(self, context: Context, pos: carla.Transform):
        # controlliamo che i parametri dati siano giusti
        if not type(context) == Context:
            raise TypeError('expected accSimulation.Context object')
        if not type(pos) == carla.Transform:
            raise TypeError('expected carla.Transform object')

        self._context = context
        self._pos = pos
        self._followSimulation = Event()
        self._followSimulation.clear()
        #self._lead = _Lead(self._context, self._pos)
        self._ego = _Ego(self._context, self._pos)

    def follow(self):
        if not self._followSimulation.is_set():
            self._followSimulation.set()
            self._follow_thread = Thread(target=self._ego.trdPerson, args=(self._followSimulation,))
            self._follow_thread.start()

    def stopFollow(self):
        if self._followSimulation.is_set():
            self._followSimulation.clear()
            self._follow_thread.join()
    
    def start(self):
       #self._lead.engine()
       self._ego.engine() 
    
    def clean(self):
        #TODO a quanto pare i carla.Vehicle non sono distruttibili
        self._ego.destroy() 
        self._lead.destroy()

#############################################################
# Module's private classes
#############################################################
class _Lead:
    def __init__(self, context, pos):
        models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
        blueprints = []
        for vehicle in context.world.get_blueprint_library().filter('*vehicle*'):
            if any(model in vehicle.id for model in models):
                blueprints.append(vehicle)
        self._context = context
        self._pos = pos
        # questo serve se voglio avere un solo veicolo fisso
        #blueprint = self._context.world.get_blueprint_library().find('vehicle.audi.a2')
        self._vehicle = self._context.world.spawn_actor(random.choice(blueprints), self._pos)

    def destroy(self):
        self._vehicle.destroy()

    def engine(self):
        # questo mi permette di usare l'autopilot per l'automobile
        tm = self._context.client.get_trafficmanager(8000)
        tm_port = tm.get_port()
        self._vehicle.set_autopilot(tm_port)

    def _stop_engine(self):
        # TODO trovare il modo per fermare il veicolo quando necessario
        pass

    def _destroy_engine(self):
        pass

class _Ego:
    def __init__(self, context, pos):
        self._context = context
        self._pos = pos
        # spawn ego
        ego_spawn = self._pos
        ego_spawn.location -= 10*(self._pos.get_forward_vector())
        ego_bp = self._context.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        ego_bp.set_attribute('role_name', 'hero')
        self._ego = self._context.world.spawn_actor(ego_bp, ego_spawn)
        self._init_sensors()
    
    def destroy(self):
        #TODO su veicolo non funziona, forse distrugge solo sensori
        self._destroy_sensors()
        self._ego.destroy()

    # generalmente inutile ma mi da un buon wrap up della lista di sensori che abbiamo 
    def _init_sensors(self):
        self._camera = self._Camera(self, self._context)
        self._radar = self._Radar(self, self._context)

    def trdPerson(self, followMe: Event):
        spectator = self._context.world.get_spectator()
        while followMe.is_set():
            # per posizionare lo spettator e in posizione del veicol
            transform = self._ego.get_transform()
            #y va verso il culo della macchina
            transform.location.z += 2
            spectator.set_transform(transform)
            time.sleep(0.05)


    def start_sensors(self):
        # TODO deve sincronizzare i dati dei due sensori allo stesso tick
        pass
    
    def stop_sensors(self):
        self._camera.stop()
        self._radar.stop()

    def _destroy_sensors(self):
        self._camera.destroy()
        self._radar.DestroyActor()
    
    def engine(self):
        map = self._context.world.get_map()
        def iter_control():
            waypoint0 = map.get_waypoint(self._ego.get_location(),
                                    project_to_road=True, 
                                    lane_type=(carla.LaneType.Driving | 
                                               carla.LaneType.Sidewalk))
            while True:
                waypoint1 = map.get_waypoint(self._ego.get_location(),
                                        project_to_road=True, 
                                        lane_type=(carla.LaneType.Driving | 
                                                carla.LaneType.Sidewalk))
                steer_angle = waypoint1.transform.get_right_vector(
                ).get_vector_angle(waypoint0.transform.get_right_vector())
                steer_angle = -1*steer_angle if not math.isnan(steer_angle) else 0
                print(steer_angle)
                waypoint0 = waypoint1
                self._ego.apply_ackermann_control(carla.VehicleAckermannControl(
                    speed = 10, acceleration = 50, steer=steer_angle))
        
        self._controlThread = Thread(target=iter_control, args=())
        self._controlThread.start()
    
    class _Camera:
        def __init__(self, up, context: Context):
            self._up = up
            self._context = context
            # impostiamo l'altezza a cui posizionare la camera z=1.5 dovrebbe impostare la camera sul cruscotto (?)
            camera_init_trans = carla.Transform(carla.Location(z=1.6))
            # otteniamo il blueprint per generare un sensore camera RGB
            camera_bp = self._context.world.get_blueprint_library().find('sensor.camera.rgb')
            # la risoluzione della camera deve avere dimensioni perfettamente divisibili per 64
            # altrimenti a causa di un bug non riesce ad interlacciare perfettamente l'immagine
            camera_bp.set_attribute("image_size_x", '512')
            camera_bp.set_attribute("image_size_y", '384')
            camera_bp.set_attribute("enable_postprocess_effects", "True")
            # generiamo la camera            
            self._camera = self._context.world.spawn_actor(camera_bp, camera_init_trans, attach_to=self._up._ego)
        
        def destroy(self):
            #TODO su veicolo non funziona, forse distrugge solo sensori
            self._camera.DestroyActor()

        def listen(self):
            #TODO va aggiustata
                """
                def stream_video(image):
                    #TODO questa funzione è bloccante, non posso quindi elaborare l'immagine successiva
                    cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('Camera Stream', image.width, image.height)

                    while not (cv2.waitKey(1) & 0xFF == ord('q')):
                        cv2.imshow('Video in tempo reale', image.raw_data)
                    # Chiudi la finestra
                    cv2.destroyAllWindows()
                """
                def camera_callback(image):
                    #stream_video(image) 
                    #image.save_to_disk('out/%06d.png' % image.frame)
                    pass
                self._camera.listen(lambda image: camera_callback(image))
        
        def stop(self):
            self._camera.stop()
        

    class _Radar:
        def __init__(self, up, context : Context):
            # Get radar blueprint 
            self._context = context
            self._up = up
            radar_bp = self._context.world.get_blueprint_library().find('sensor.other.radar')
            # Set Radar attributes, by default are:
            radar_bp.set_attribute('horizontal_fov', '30') # degrees 
            radar_bp.set_attribute('vertical_fov', '30') # degrees 
            radar_bp.set_attribute('points_per_second', '1500')
            radar_bp.set_attribute('range', '100') # meters 
            # Spawn the radar 
            self._radar = self._context.world.spawn_actor(radar_bp, carla.Transform(carla.Location(z=2)), attach_to=self._up._ego)
        
        def destroy(self):
            #TODO su veicolo non funziona, forse distrugge solo sensori
            self._radar.DestroyActor()
            
        def listen(self):
            def stream_video(self):
                    pass

        # la funzione da modificare per gestire i dati del radar    
            def radar_callback(sensor_data):
                for detection in sensor_data:
                    print('depth: ' + str(detection.depth)) # meters 
                    print('azimuth: ' + str(detection.azimuth)) # rad 
                    print('altitude: ' + str(detection.altitude)) # rad 
                    print('velocity: ' + str(detection.velocity)) # m/s 
            self._radar.listen(lambda radar_data: radar_callback(radar_data))
        
        def stop(self):
            self._radar.stop()