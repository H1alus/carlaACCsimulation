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
        y, cPoint = self.closestPoint()
        desired = np.deg2rad(cPoint.rotation.yaw)
        theta = np.deg2rad(self.ego.get_transform().rotation.yaw)
        sigma = np.angle(np.exp(1j * (theta - desired)))
        v = self.ego.get_velocity()
        v = np.array([v.x, v.y, v.z])
        v = np.sqrt(v.dot(v))
        v_st = v/np.cos(desired)
        y_st = y + sigma*Tracker.WHEELBASE 
        delta = -(sigma + np.arctan(Kp * y_st/(v_st + 1e-3)))
        delta = ((delta + 1) % 2) - 1
        return delta
    
    def keepTrack(self):
        """
        Applies both the steering from stanley and the ability to keep the car
        on the center of the track
        ## parameters:
        None

        ## returns:
        steering angle (rad)
        """
        Kp = 3.4
        step = 0.1
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
        egoTransform = self.ego.get_transform()
        egoTransform.location = egoTransform.location + step * (cPoint.location - egoTransform.location)
        self.ego.set_transform(egoTransform)
        delta = ((delta + 1) % 2) - 1
        return delta
    
    def desired_speed(self,speed):
        """
        convert speed (Km/h) to the throttle parameter
        based on current speed

        ## parameteres:
        vehicle: carla.Vehicle object, the vehicle to apply the 

        speed: target speed we want to obtain

        ## returns:
        throttle: float - the required throttle to achieve the required speed
        """
        v = self.ego.get_velocity()
        v = np.array([v.x, v.y, v.z])
        v = np.sqrt(v.dot(v))
        _, cPoint = self.closestPoint()
        idx = self.wps.tolist().index(cPoint)

        def speeding(speed):
            if v < speed:
                throttle = 0.9
                brake = 0
            elif v > speed + 0.166666667:
                throttle = 0
                brake = max(((v - speed + 1) % 2) + 1 - 0.3, 0)
            else:
                throttle = 0.4
                brake = 0


            return throttle, brake
        if speed > 60:
            if np.abs(np.angle(np.exp(1j*np.deg2rad(cPoint.rotation.yaw))) - np.angle(
                np.exp(1j*np.deg2rad(self.wps[(idx + 15)%len(self.wps)].rotation.yaw)))) > 1e-2:
                return speeding(16.67)
        return speeding(speed/3.6)