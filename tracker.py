import numpy as np
import carla


class Tracker:
    """
    This class implements a set of methods to keep track of the route
    The steer method uses the Stanley auto steering
    """
    WHEELBASE = 2.847
    def __init__(self, vehicle : carla.Vehicle, route):
        # in the context of this class the vehicle becomes our ego
        self.ego = vehicle
        self.wps = np.array([wp[0].transform for wp in route])
        #self.integral = 0
        #self.prev_error = 0
        #self.prev_angle = 0

    def closestPoint(self):
        """
        A uinque function to return both the lateral error 
        and the closest point

        ## returns:
        lateralError: float - distance between y coords between
        waypoint and location of the vehicle

        cPoint: float - closest waypoint to vehicle
        """
        vlocation = self.ego.get_location()
        distances = [wp.location.distance(vlocation) for wp in self.wps]
        closest = min(distances)
        idx = distances.index(closest)
        cPoint = self.wps[idx]
        lateralError = vlocation.y - cPoint.location.y
        return lateralError, cPoint

    def steer(self):
        """
        This function computes the steering angle required
        
        Call this whenever you call carla.Vehicle.apply_control()
        in order to apply the steering angle required for the vehicle

        ## returns:
        angle: float limited in interval [-1, 1]
        """
        Kp = 3.4
        # Ku = 3.4
        # Tu = 1
        # Kp = 0.6*Ku
        # Ki = 2 * Kp/Tu
        # Kd = Kp*Tu/8
        y, cPoint = self.closestPoint()
        delta = np.deg2rad(cPoint.rotation.yaw)
        theta = np.deg2rad(self.ego.get_transform().rotation.yaw)
        sigma = np.angle(np.exp(1j * (theta - delta)))
        v = self.ego.get_velocity()
        v = np.array([v.x, v.y, v.z])
        v = np.sqrt(v.dot(v))
        v_st = v/np.cos(delta)
        y_st = y + sigma*AutoSteer.WHEELBASE
        #proportional = Kp * y_st
        # self.integral += y_st
        # integral = Ki * self.integral
        # derivative = Kd * (y_st - self.prev_error)
        # self.prev_error = y_st
        # PID = proportional + integral + derivative
        delta = -(sigma + np.arctan(Kp * y_st/(v_st + 1e-3)))
        delta = ((delta + 1) % 2) - 1
        return delta
    
    def keepTrack(self):
        Kp = 3.4
        y, cPoint = self.closestPoint()
        delta = np.deg2rad(cPoint.rotation.yaw)
        theta = np.deg2rad(self.ego.get_transform().rotation.yaw)
        sigma = np.angle(np.exp(1j * (theta - delta)))
        v = self.ego.get_velocity()
        v = np.array([v.x, v.y, v.z])
        v = np.sqrt(v.dot(v))
        v_st = v/np.cos(delta)
        y_st = y + sigma*Tracker.WHEELBASE
        delta = -(sigma + np.arctan(Kp * y_st/(v_st + 1e-3)))
        delta = ((delta + 1) % 2) - 1
        update_step = 0.1
        egoTransform = self.ego.get_transform()
        egoTransform.location = egoTransform.location + update_step * (cPoint.location - egoTransform.location)
        self.ego.set_transform(egoTransform)
        return delta