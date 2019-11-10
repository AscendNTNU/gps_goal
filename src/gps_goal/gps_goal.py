#!/usr/bin/python
import rospy
import math
import actionlib
import tf
import ascend_msgs.msg

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix, Imu

def DMS_to_decimal_format(lat,long):
  # Check for degrees, minutes, seconds format and convert to decimal
  if ',' in lat:
    degrees, minutes, seconds = lat.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if lat[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    lat = degrees + minutes/60 + seconds/3600
  if ',' in long:
    degrees, minutes, seconds = long.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if long[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    long = degrees + minutes/60 + seconds/3600

  lat = float(lat)
  long = float(long)
  rospy.loginfo('Given GPS goal: lat %s, long %s.' % (lat, long))
  return lat, long

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose = rospy.wait_for_message("/mavros/global_position/global", NavSatFix)
  origin_lat = origin_pose.latitude
  origin_long = origin_pose.longitude
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long

def calc_circle(x,y,r):
  x = cx + r * cos(a)
  y = cy + r * sin(a)





def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  try: 
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
    hypotenuse = distance = g['s12'] # access distance
    rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
    azimuth = g['azi1']
    rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))
    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
    # Convert azimuth to radians
    azimuth = math.radians(azimuth)
    y = adjacent = -math.cos(azimuth) * hypotenuse
    x = opposite = math.sin(azimuth) * hypotenuse
    rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

  except NameError:
    rospy.loginfo("No orientation")
  return x, y

def active_callback():
    print("Goal active!")

def feedback_callback(feedback):
    pass
    #print("Feedback - " + "Current state: " + feedback.state.data + "\n Current pose: " + str(feedback.pose_stamped))
    # Do something with the pose: feedback.pose_stamped

def done_callback(state, result):
    pass
    #print("Finshed with state: " + str(state) + "\nFinal Fluid state: " + result.state.data + "\n Final pose: " + str(result.pose_stamped))
    # Do something with the pose: feedback.pose_stamped  

class GpsGoal():
  def __init__(self):
    rospy.init_node('gps_goal')

    rospy.loginfo("Connecting to move_base...")
    self.client = actionlib.SimpleActionClient('fluid_operation', ascend_msgs.msg.FluidAction)
    self.client.wait_for_server()
    rospy.loginfo("Connected.")

    rospy.Subscriber('gps_goal_pose', PoseStamped, self.gps_goal_pose_callback)
    #rospy.Subscriber('/airsim_node/PX4/imu/Imu', Imu, self.orientation_callback)
    #rospy.Subscriber('gps_goal_fix', NavSatFix, self.gps_goal_fix_callback)
    orientation = rospy.wait_for_message('/airsim_node/PX4/imu/Imu', Imu)
    self.orientation_callback(orientation)
    # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
    self.origin_lat, self.origin_long = get_origin_lat_long()
    waypoints = self.calc_waypoint(self.origin_lat, self.origin_long, 3)
    self.publish_goal(waypoints, yaw=0, roll=0, pitch=0)

  def do_gps_goal(self, goal_lat, goal_long, z=10, yaw=0, roll=0, pitch=0):
    pass
    # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
    #x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)

    # Create move_base goal
    #self.publish_goal(waypoints, yaw=yaw, roll=roll, pitch=pitch)
    #self.publish_goal(x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch)
  
  def orientation_callback(self, msg):
    rospy.loginfo("Got orientation")
    rospy.loginfo(msg.orientation)
    quaternion = (
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w)
    self.orientation = tf.transformations.euler_from_quaternion(quaternion)
    self.orientation = math.degrees(self.orientation[2])
    rospy.loginfo("Yaw: {:.3f}".format(self.orientation))
  
  def calc_waypoint(self, origin_lat, origin_long, r):
    pylon1 = [47.6415639, -122.1401812]
    pylon2 = [47.641366,  -122.1401812]

    x1, y1 = calc_goal(pylon1[0], pylon1[1], origin_lat, origin_long)
    x2, y2 = calc_goal(pylon2[0], pylon2[1], origin_lat, origin_long)

    possible_waypoints = [
                      [x1, y1 + r], 
                      [x1 - r, y1],
                      [x2 - r, y2],
                      [x2, y2 - r],
                      [x2 + r, y2],
                      [x1 + r, y1]
                      ]
    rospy.loginfo(possible_waypoints)
    return possible_waypoints


  def gps_goal_pose_callback(self, data):
    lat = data.pose.position.y
    long = data.pose.position.x
    z = data.pose.position.z
    euler = tf.transformations.euler_from_quaternion(data.pose.orientation)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    self.do_gps_goal(lat, long, z=z, yaw=yaw, roll=roll, pitch=pitch)

  def gps_goal_fix_callback(self, data):
    self.do_gps_goal(data.latitude, data.longitude)

  def publish_goal(self, waypoints, yaw=0, roll=0, pitch=0):
    # Create move_base goal
    goal = ascend_msgs.msg.FluidGoal()
    #goal.setpoint.x = x
    #goal.setpoint.y = y
    #goal.setpoint.z =  z
    #quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    #goal.target_pose.pose.orientation.x = quaternion[0]
    #goal.target_pose.pose.orientation.y = quaternion[1]
    #goal.target_pose.pose.orientation.z = quaternion[2]
    #goal.target_pose.pose.orientation.w = quaternion[3]
    #rospy.loginfo('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
     #       (x, y, yaw))
    #rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

    # Send goal
    

    #Should send this action when we have calculated waypoints
    goal.mode.data = "take_off"

    print("Sending goal")
    # Sends the goal to the action server.
    self.client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
     # Waits for the server to finish performing the action.
    self.client.wait_for_result()

    goal = ascend_msgs.msg.FluidGoal()
    goal.mode.data = "move"
    goal.setpoint.z = 2
    for point in waypoints:
      goal.setpoint.x = point[0]
      goal.setpoint.y = point[1]
      print("x: " +str(point[0])+ ", y: " +str(point[1]) + ", z: " +str(2))
      rospy.loginfo("Sending waypoint")
      self.client.send_goal(goal, active_cb=active_callback, feedback_cb=feedback_callback, done_cb=done_callback)
      self.client.wait_for_result()
      rospy.loginfo("Wait for waypoint finish")
    






    #rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    #status = self.move_base.get_goal_status_text()
    #if status:
     # rospy.loginfo(status)

    
    # Wait for goal result
   # rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))

#@click.command()
#@click.option('--lat', prompt='Latitude', help='Latitude')
#@click.option('--long', prompt='Longitude', help='Longitude')
#@click.option('--roll', '-r', help='Set target roll for goal', default=0.0)
#@click.option('--pitch', '-p', help='Set target pitch for goal', default=0.0)
#@click.option('--yaw', '-y', help='Set target yaw for goal', default=0.0)
def cli_main(lat, long, roll, pitch, yaw):
  """Send goal to move_base given latitude and longitude

  \b
  Two usage formats:
  gps_goal.py --lat 43.658 --long -79.379 # decimal format
  gps_goal.py --lat 43,39,31 --long -79,22,45 # DMS format
  """
  gpsGoal = GpsGoal();

  # Check for degrees, minutes, seconds format and convert to decimal
  lat, long = DMS_to_decimal_format(lat, long)
  gpsGoal.do_gps_goal(lat, long, roll=roll, pitch=pitch, yaw=yaw)


def ros_main():
  gpsGoal = GpsGoal();
  rospy.spin()

if __name__ == '__main__':
  cli_main()
