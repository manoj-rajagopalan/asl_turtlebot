#!/usr/bin/env python

from enum import Enum

import rospy
from asl_turtlebot.msg import DetectedObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP_SIGN = 3
    CROSS = 4
    MANUAL = 6


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.1)
        self.theta_eps = rospy.get_param("~theta_eps", 0.3)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5)

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
        self.x_g = self.x
        self.y_g = self.y
        self.theta_g = self.theta

        # Current mode
        self.mode = Mode.IDLE
        self.prev_mode = None  # For printing purposes
        self.dist_to_stop = float('inf')

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)

        #- # Command vel (used for idling)
        #-= self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
            # Event loop will begin moving vehicle


    ########## SUBSCRIBER CALLBACKS ##########

    # /gazebo/model_states
    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    # /move_base_simple/goal
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

    # /nav_pose
    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta

    # /detector/stop_sign
    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        self.dist_to_stop = msg.distance


    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def idle_action(self):
        """ Tricks the controller into sending zero velocity by providing the
        current pose as its goal"""
        assert self.mode == Mode.IDLE
        self.publish_current_pose_to_controller()

        # transit to POSE mode if goal was received
        if not self.is_close_to(self.x_g, self.y_g, self.theta_g):
            self.mode = Mode.POSE

    def pose_action(self):
        """ sends the current desired pose to the pose controller """
        assert self.mode == Mode.POSE
        if self.is_close_to_goal():
            rospy.loginfo("POSE: Close to goal, switching to IDLE")
            # stop the robot ...
            self.publish_current_pose_to_controller()
            # ... and keep it so
            self.x_g = self.x
            self.y_g = self.y
            self.theta_g = self.theta
            self.mode = Mode.IDLE

        elif self.is_close_to_stop_sign():
            rospy.loginfo("POSE: close to stop sign, stopping and swithching to STOP")
            self.stop_sign_start = rospy.get_rostime()
            self.publish_current_pose_to_controller() # stop the robot
            self.mode = Mode.STOP_SIGN

        else:
            self.publish_goal_to_controller()
            # and maintain Mode.POSE state

    def stop_sign_action(self):
        assert self.mode == Mode.STOP_SIGN
        if rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time):
            rospy.loginfo("STOP_SIGN: wait finished, switching to CROSS")
            self.publish_goal_to_controller() # get moving ...
            self.mode = Mode.CROSS # ... and ignore stop sign
        else:
            self.publish_current_pose_to_controller() # stay stopped
            # maintain self.mode = Mode.STOP_SIGN

    def cross_action(self):
        assert self.mode == Mode.CROSS
        assert self.is_close_to_stop_sign()
        self.publish_goal_to_controller() # move unconditionally
        if not self.is_close_to_stop_sign():
            rospy.loginfo('CROSS: past stop sign, switching to POSE')
            # subsequently begin caring about stop sign
            self.mode = Mode.POSE
        #else continue ignoring stop sign in Mode.CROSS

    ########## MOVE/STOP MESSAGES TO CONTROLLER ##########

    def publish_goal_to_controller(self):
        """ Publishes true goal to controller in order to cause movement towards it"""
        goal_pose = Pose2D()
        goal_pose.x = self.x
        goal_pose.y = self.y
        goal_pose.theta = self.theta
        self.pose_goal_publisher.publish(goal_pose)

    def publish_current_pose_to_controller(self):
        """ Publishes current pose as goal to controller in order to stop the vehicle"""
        current_pose = Pose2D()
        current_pose.x = self.x
        current_pose.y = self.y
        current_pose.theta = self.theta
        self.pose_goal_publisher.publish(current_pose)

    ########## USEFUL PREDICATES ##########

    def is_close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(theta - self.theta) < self.params.theta_eps

    def is_close_to_goal(self):
       return self.is_close_to(self.x_g, self.y_g, self.theta_g)

    def is_close_to_stop_sign(self):
        return self.dist_to_stop > 0 and self.dist_to_stop < self.params.stop_min_dist

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
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
            self.idle_action()

        elif self.mode == Mode.POSE:
            self.pose_action()

        elif self.mode == Mode.STOP_SIGN:
            self.stop_sign_action()

        elif self.mode == Mode.CROSS: # intersection
            self.cross_action()

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
