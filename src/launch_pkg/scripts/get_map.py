#!/usr/bin/env python3
# Author: DuyKhang


from sensor_msgs.msg import LaserScan , Image , PointCloud2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int8

# from sti_msgs.msg import * # Status_port Driver_respond, Driver_respond
# from message_pkg.msg import *

import roslaunch
import rospy
import string
import time
import os

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # -- variable
        self.process = 0
        self.time_pre = time.time()

    def start(self):
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1  

    def start_and_wait(self, timeWait): # second - Dung cho cac node ko pub.
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1
            self.time_pre = time.time()
            return 0

        elif (self.process == 1): # - Wait
            t = (time.time() - self.time_pre)%60
            if (t > timeWait):
                self.process = 2
            return 0

        elif (self.process == 2): # - Wait
            return 1

class launcher():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('launcher', anonymous=False)
        self.rate = rospy.Rate(10)

        self.count_node = 0
        self.notification = ''
        self.step = 0
        self.timeWait = 0.4 # s

        # self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
        # self.stausLaunch = Status_launch()

        # -- module - firstWork. => xoa du lieu log file cua ros
        self.path_firstWork = rospy.get_param('path_firstWork', '')
        self.launch_firstWork = Launch(self.path_firstWork)
        rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
        self.is_firstWork = 0
        self.count_node += 1

        # -- rosserial
        self.path_core = rospy.get_param('path_serial', '')
        self.launch_core = Launch(self.path_core)
        rospy.Subscriber('/imu', Imu, self.callBack_core)
        self.is_core = 0
        self.count_node += 1

        # -- rplidar.
        self.path_lidar = rospy.get_param('path_lidar', '')
        self.launch_lidar = Launch(self.path_lidar)
        rospy.Subscriber('/scan', LaserScan, self.callBack_lidar)
        self.is_lidar = 0
        self.count_node += 1

        # -- slam-gmapping
        self.path_slam = rospy.get_param('path_slam', '')
        self.launch_slam = Launch(self.path_slam)
        self.is_slam = 0
        self.count_node += 1

        # -- kinematic.
        self.path_kinematic = rospy.get_param('path_kinematic', '')
        self.launch_kinematic = Launch(self.path_kinematic)
        rospy.Subscriber('/odom_encoder', Odometry, self.callBack_kinematic)
        self.is_kinematic = 0
        self.count_node += 1

        # -- odom
        self.path_ekf = rospy.get_param('path_ekf', '')
        self.launch_ekf = Launch(self.path_ekf)
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.callBack_ekf)
        self.is_ekf = 0
        self.count_node += 1

        # -- rviz.
        self.is_display = rospy.get_param('rviz', 1);
        self.path_rviz= rospy.get_param('path_rviz', '')
        self.launch_rviz = Launch(self.path_rviz)
        self.is_rviz = 0
        self.count_node += 1

    def callBack_firstWork(self, data):
        self.is_firstWork = 1

    def callBack_core(self, data):
        self.is_core = 1

    def callBack_lidar(self, data):
        self.is_lidar = 1

    def callBack_kinematic(self, data):
        self.is_kinematic = 1

    def callBack_ekf(self, data):
        self.is_ekf = 1

    def run(self):
        while not rospy.is_shutdown():
            # print "runing"
            # -- firstWork
            if (self.step == 0):
                self.notification = 'launch_firstWork'
                self.launch_firstWork.start()
                if (self.is_firstWork == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- rplidar 
            elif (self.step == 1):
                self.notification = 'launch_lidar'
                self.launch_lidar.start()
                # self.step = 2
                if (self.is_lidar == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- main board stm32
            elif (self.step == 2):
                self.notification = 'launch_serial'
                self.launch_core.start()
                # self.step = 3
                if (self.is_core == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- kinematic
            elif (self.step == 3):
                self.notification = 'launch_kinematic'
                self.launch_kinematic.start()
                # self.step = 4
                if (self.is_kinematic == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- robot_pose_ekf
            elif (self.step == 4):
                self.notification = 'launch_ekf'
                self.launch_ekf.start()
                # self.step = 5
                if (self.is_ekf == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- navigation
            elif (self.step == 5):
                self.notification = 'launch_slam'
                sts = self.launch_slam.start_and_wait(3.)
                if (sts == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- rviz
            elif (self.step == 6):
                self.notification = 'launch_rviz'
                sts = self.launch_rviz.start_and_wait(3.)
                if (sts == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- Completed
            elif (self.step == 7):
                self.notification = 'Completed!'

            # -- -- PUBLISH STATUS
            # self.stausLaunch.persent = int((self.step/self.count_node)*100.)
            # self.stausLaunch.persent = int((self.step/7)*100.)
            # self.stausLaunch.position = self.step
            # self.stausLaunch.notification = self.notification
            # self.pub_stausLaunch.publish(self.stausLaunch)
            # time.sleep(0.1)
            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = launcher()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()
