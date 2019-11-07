from geographiclib.geodesic import Geodesic
import math

#z = yaw
#y = pitch?
#x = roll?


goal_lat = 47.6419792175
goal_long = -122.140106201

origin_lat = 47.6419792175
origin_long = -122.140174866

#latitude: 47.6419792175
#longitude: -122.140174866
#altitude: 123.333000183

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  print("The distance from the origin to the goal is {:.3f} m.".format(distance))
  azimuth = g['azi1']
  print("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))
   # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  print("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

  return x, y
print(calc_goal(origin_lat, origin_long, goal_lat, goal_long))