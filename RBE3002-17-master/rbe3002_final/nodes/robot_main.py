#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class Robot_Main: 
    def __init__(self):

        self.px = 0
        self.py = 0
        self.pth = 0
        self.odom = Odometry()
        self.current_goal = False
        self.doneEx = False
        rospy.init_node('robot_main', anonymous=True)

        self.pathServ = rospy.ServiceProxy('plan_path', GetPlan)       
        #self.start = PoseStamped() 
        #self.goal = PoseStamped() 
        self.tolerance = .05
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.make_plan)  
        self.return_sub = rospy.Subscriber('Return', PoseStamped, self.make_plan)
        self.frontier_sub = rospy.Subscriber('Centroids/Best', PoseStamped, self.update_goal)

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry,  queue_size=100)

        #self.testStartpub = rospy.Publisher('testStartPub', PoseStamped, queue_size=10)
        #self.testGoalpub = rospy.Publisher('testGoalPub', PoseStamped, queue_size=10)
        rospy.wait_for_service("plan_path")
        print("Robot Init")
        rospy.sleep(1.0)

    def update_goal(self,msg):
        goal = PoseStamped()
        goalX = msg.pose.position.x
        goalY = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (_, _, yaw) = euler_from_quaternion(quat_list)
        goalTh = yaw 
        goal.pose.position.x = goalX
        goal.pose.position.y = goalY
        goal.pose.position.z = 0
        quat2 = quaternion_from_euler(0,0,goalTh)
        goal.pose.orientation.x = quat2[0]
        goal.pose.orientation.y = quat2[1]
        goal.pose.orientation.z = quat2[2]
        goal.pose.orientation.w = quat2[3]
        if not (goal.pose.position.x == 0 and goal.pose.position.y == 0):
            self.current_goal = goal
        return goal

    def make_plan(self, msg):
        start = PoseStamped()
        goal = PoseStamped()
        self.doneEx = True
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        
        quat = quaternion_from_euler(0,0,self.pth)
        start.pose.orientation.x = quat[0]
        start.pose.orientation.y = quat[1]
        start.pose.orientation.z = quat[2]
        start.pose.orientation.w = quat[3]

        #goal values
        goalX = msg.pose.position.x
        goalY = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (_, _, yaw) = euler_from_quaternion(quat_list)
        goalTh = yaw 
        goal.pose.position.x = goalX
        goal.pose.position.y = goalY
        goal.pose.position.z = 0
        quat2 = quaternion_from_euler(0,0,goalTh)
        goal.pose.orientation.x = quat2[0]
        goal.pose.orientation.y = quat2[1]
        #print("New Goal", goal)
        #rospy.wait_for_service("plan_path")
        #self.testStartpub.publish(start)
        #self.testGoalpub.publish(goal)
        #print(start,goal,self.tolerance)
        print("true goal pose", goal.pose.position)
        aPlan = self.pathServ(start,goal,self.tolerance)

        b_first = False
        #print("length", len(aPlan.plan.poses))
        for waypoint in aPlan.plan.poses:
            #print("pose", waypoint.pose.position)
            if b_first:
                self.go_to(waypoint)
            b_first = True
        return 1
    
    def make_plan_centroids(self, msg):
        start = PoseStamped()
        goal = PoseStamped()

        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0
        
        quat = quaternion_from_euler(0,0,self.pth)
        start.pose.orientation.x = quat[0]
        start.pose.orientation.y = quat[1]
        start.pose.orientation.z = quat[2]
        start.pose.orientation.w = quat[3]

        #goal values
        goal = msg
        #rospy.wait_for_service("plan_path")
        #self.testStartpub.publish(start)
        #self.testGoalpub.publish(goal)
        #print(start,goal,self.tolerance)
        print("true goal pose", goal.pose.position)
        aPlan = self.pathServ(start,goal,self.tolerance)

        b_first = False
        #print("length", len(aPlan.plan.poses))
        for waypoint in aPlan.plan.poses[0:int(.5*len(aPlan.plan.poses))]:
            #print("pose", waypoint.pose.position)
            if b_first and not self.doneEx:
                self.go_to(waypoint)
            b_first = True
        return 1

    def send_speed(self, linear_speed, angular_speed):
        #print("send_speed")
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        #Control adjustment for twist and linear velocity
        msg_cmd_vel = Twist()


        kp = .2
        spin = self.odom.twist.twist.angular.z
        spin_error = angular_speed - spin
        if spin_error > 0:
            spin_adjust = kp*spin_error
        elif spin_error < 0:
            spin_adjust = -kp*spin_error
        else:
            spin_adjust = 0

        if linear_speed == 0:
            spin_adjust = 0

        vel = math.sqrt(self.odom.twist.twist.linear.x**2 + self.odom.twist.twist.linear.y**2)
        vel_error = linear_speed - vel
        #print(vel_error)
        if vel_error > 0:
            vel_adjust = kp*vel_error
        elif vel_error < 0:
            vel_adjust = -kp*vel_error
        else:
            vel_adjust = 0

        ### Make a new Twist message
        # Linear velocity
        vel_adjust = 0
        msg_cmd_vel.linear.x = linear_speed + vel_adjust
        #print('linear speed', linear_speed)
        #print('vel_adjust', vel_adjust)
        if linear_speed == 0:
            msg_cmd_vel.linear.x = 0
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        spin_adjust = 0
        msg_cmd_vel.angular.z = angular_speed + spin_adjust
        if angular_speed == 0:
            msg_cmd_vel.angular.z = 0
        ### Publish the message
        #print(msg_cmd_vel)
        self.pub.publish(msg_cmd_vel)
        ## pass # delete this when you implement your code

    
        
    def drive(self, distance, linear_speed, start_time):
        print("drive")
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT

        #saving the initial pose
        startx = self.px
        starty = self.py
        #Takes in initial heading and goal location to course correct
        startth = self.pth
        goalx = distance * math.cos(self.pth) + startx
        goaly = distance * math.sin(self.pth) + starty


        #drive forward
        self.send_speed(linear_speed, 0.0)
        separation = 1
        should_be_time = distance / linear_speed
        start_time = time.time()
        print('separation start while')
        while separation >.06 and ((time.time() - start_time) < should_be_time):
            #rospy.sleep(.075)
            #check threshold
            #adjusts the heading by an amount proportional to how far off it is
            goal_heading = math.atan2(goaly-self.py,goalx-self.px)
            heading_error = goal_heading - self.pth
            #accepts a heading error if the robot is looping across the heading boundary
            if abs(heading_error) > 6.0:
                heading_error = 0
            #print(goal_heading)
            #self.send_speed(linear_speed, 3 * heading_error)
            separation = distance - math.sqrt((self.px-startx)**2.0 + (self.py-starty)**2.0)
            print('separation ', separation)
        #stop
        self.send_speed(0, 0)
       # pass # delete this when you implement your code



    def rotate(self, angle, aspeed):
        print("rotate")
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        #initial pose
        startth = self.pth
        #proportion of the circle to travel
        diff = angle % (2*math.pi)
        #making sure the proportion is optimal and the direction is correct
        if abs(diff) > math.pi:
            diff = (angle % (2*math.pi)) - (2*math.pi)
            self.send_speed(0, aspeed*-1)
        else:
            self.send_speed(0,aspeed)

        #finding the ending pose
        goal = startth + diff
        if abs(goal / math.pi) > 1:
            overshoot = abs(goal) - math.pi
            if goal < 0:
                goal = math.pi-overshoot
            elif goal > 0:
                goal = -math.pi + overshoot

        separation = 1
        #checks for completion
        while abs(separation) >.05:
            #rospy.sleep(.075)
            #update threshold
            print("angle separation", separation)
            separation = goal - self.pth

        #stop
        self.send_speed(0, 0)
        #pass # delete this when you implement your code

    def go_to(self, msg):
        #print("goto")
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #initial values
        startx = self.px
        starty = self.py
        startth = self.pth
        #goal values
        goalX = msg.pose.position.x
        goalY = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        goalTh = yaw

        #the linear differences
        diffx = goalX-startx
        diffy = goalY-starty

        #Finds the desired heading and the moves to the pose
        intTheta = math.atan2(diffy, diffx)
        self.rotate(intTheta-startth,.2)
        #time.sleep(2.0)
        self.drive(math.sqrt(diffy**2 + diffx**2),.1, time.time())
        #time.sleep(2.0)
        self.rotate(goalTh-intTheta,.2)
        #time.sleep(2.0)

        
    def update_odometry(self, msg):
        #print("update_odometry")
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.odom = msg
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
        ## pass # delete this when you implement your code



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        while not rospy.is_shutdown() and not self.doneEx:
            if self.current_goal:
                self.make_plan_centroids(self.current_goal)
        rospy.spin()


        
if __name__ == '__main__':
    Robot_Main().run()
