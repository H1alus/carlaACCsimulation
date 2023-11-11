# carlaACCsimulation, a simulator for ACC systems, built around CARLA simulator
#     Copyright (C) 2023  Vittorio Folino Rocco Depietra Giuseppe Sansevero

#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.

#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.

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
