#!/usr/bin/env python

from enum import Enum

import rospy
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf

kRoadBlockLandmark = 'clock'

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = False # rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = True # rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = True # rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.1)
        self.theta_eps = rospy.get_param("~theta_eps", 0.3)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 2.0)

        # Minimum confidence for object detection
        self.min_confidence = 0.9

        # Time taken to cross an intersection
        self.crossing_time = rospy.get_param("~crossing_time", 3.)

        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist, crossing_time = {}, {}, {}".format(self.stop_time, self.stop_min_dist, self.crossing_time))


class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # Current mode
        self.mode = Mode.MANUAL
        self.prev_mode = None  # For printing purposes

        # Landmarks
        self.landmarks = {}
        self.colors = {
            "dog": [1.0, 0.0, 0.0],
            "cow": [1.0, 0.0, 0.0],
            "bird": [1.0, 0.0, 0.0],
            "horse": [1.0, 0.0, 0.0],
            "chair": [0.0, 0.0, 1.0],
            "scissors": [0.0, 0.0, 1.0],
            "clock": [1.0, 1.0, 0.0],
            "donut": [1.0, 1.0, 0.0],
        }

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Command nav
        self.cmd_nav_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        # Landmark markers
        self.markers_publisher = rospy.Publisher('/markers', Marker, queue_size=10)

        # Control
        self.man_control_publisher = rospy.Publisher('/man_control', String, queue_size=10)

        ########## SUBSCRIBERS ##########

        # Stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # High-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        if self.params.rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        else:
            self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
            self.mode = Mode.NAV

        # Food detectors
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_detected_callback)

        # Order requester
        rospy.Subscriber('/delivery_request', String, self.order_callback)

        # Control
        rospy.Subscriber('/man_control', String, self.man_control_callback)

    ########## SUBSCRIBER CALLBACKS ##########

    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if self.params.mapping else "/odom"
        print("Rviz command received!")

        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (nav_pose_origin.pose.orientation.x,
                          nav_pose_origin.pose.orientation.y,
                          nav_pose_origin.pose.orientation.z,
                          nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
            self.init_stop_sign()

    def object_detected_callback(self, msg):
        for i, obj in enumerate(msg.objects):
            if obj == "stop_sign":
                continue

            dist = msg.ob_msgs[i].distance
            conf = msg.ob_msgs[i].confidence

            if dist > 0 and dist < 2.75 and conf > self.params.min_confidence:
                pose = Pose2D()
                pose.x = self.x
                pose.y = self.y
                pose.theta = self.theta

                # special-case road-block landmark
                if obj == kRoadBlockLandmark:
                    rospy.loginfo('Roadblock detected')
                    self.man_control_publisher.publish('Roadblock')
                elif not obj in self.landmarks.keys():
                    self.landmarks[obj] = pose
                    self.marker_publisher(obj, pose)
                    print "Object Detected", obj, self.landmarks[obj]

    def order_callback(self, msg):
        for order in msg.data.split(","):
            print "Order Received", order
            if order in self.landmarks.keys():
                position = self.landmarks[order]
                self.nav_pose_callback(position)
                self.cmd_nav_publisher.publish(position)
                while not self.close_to(position.x, position.y, position.theta):
                    pass
            print "Order Pickup", order

    def marker_publisher(self, name, pose):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        marker.id = len(self.landmarks.keys())
        marker.type = 2 # sphere

        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = 0.5

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = self.colors[name][0]
        marker.color.g = self.colors[name][1]
        marker.color.b = self.colors[name][2]

        self.markers_publisher.publish(marker)
        print('Published marker!')

    def man_control_callback(self, msg):
        if msg.data == "Manual":
            self.mode = Mode.MANUAL
        elif msg.data == "Auto":
            self.mode = Mode.IDLE

    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(theta - self.theta) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.man_control_publisher.publish("Stop")

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.params.crossing_time)

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" # if mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode

        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.

        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # Moving towards a desired pose
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            if self.has_crossed():
                self.mode = Mode.POSE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.MANUAL:
            self.man_control_publisher.publish("Manual")

        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
