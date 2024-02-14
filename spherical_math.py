import numpy as np
import math

def spherical_distance(r1, theta1, phi1, r2, theta2, phi2):
    return np.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * (np.cos(theta2 - theta1) * np.sin(phi2) * np.sin(phi1) + np.cos(phi2) * np.cos(phi1)))

class Spherical_math:

    def distance_cluster(points, min_distance):
        clusters = []
        
        for point in points:
            belongs_to_cluster = False
            
            for cluster in clusters:
                for cluster_point in cluster:
                    if spherical_distance(point[0], math.degrees(point[1]), math.degrees(point[2]), 
                                          cluster_point[0], math.degrees(cluster_point[1]), math.degrees(cluster_point[2]),) < min_distance:
                        cluster.append(point)
                        belongs_to_cluster = True
                        break
                
                if belongs_to_cluster:
                    break
            
            if not belongs_to_cluster:
                clusters.append([point])
        
        return clusters

    def remove_single_element_clusters(clusters):
        # Filtra i cluster che hanno più di un elemento
        return [cluster for cluster in clusters if len(cluster) > 1]
    
    def spherical_to_cartesian(r, theta, phi):
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return x, y, z
    
    def remove_ground(data, height, percentage):
        result = []
        for item in data:
            dist  = item[0]  # distanza
            alpha = item[2]  # altitude (angolo rispetto al terreno)
            arc_sin_value = math.asin(height / dist)
            min_value = arc_sin_value - (arc_sin_value * percentage / 100)
            max_value = arc_sin_value + (arc_sin_value * percentage / 100)
            if not (min_value <= alpha <= max_value):
                result.append(item)
        return result
