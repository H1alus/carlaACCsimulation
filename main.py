import carla
import random
import time
import numpy as np
import weakref
from accSimulation import Simulation, Context

context = Context()
context.client.load_world('Town04')
# se non va terminare il task rpc service da task manager
pos = carla.Transform(carla.Location(x=402.784515, y=-149.542175, z=0.281942), 
                      carla.Rotation(pitch=0.000000, yaw=-89.401421, roll=0.000000))
simulation = Simulation(context, pos)
#simulation.start()
simulation.follow()
time.sleep(2)
simulation.stopFollow()
