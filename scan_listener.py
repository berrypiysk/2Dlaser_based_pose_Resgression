#!/usr/bin/env python
# get the scan topic then turn the data into the .bin or .jpg
import argparse
import rospy
import tf
import tf.transformations as TFt
import os
#import pcl
import numpy as np
import math
from roslib import message
from std_msgs.msg import String
from std_msgs.msg import Header
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import os
# from pykeyboard import PyKeyboardEvent
import time, threading
import matplotlib.pyplot as plt

from tf.transformations import quaternion_from_euler, euler_from_quaternion

global is_Save
global dDelta     
global lastDistance

# fig0 = plt.figure(0,figsize=(8,8),dpi=100,facecolor='white')
# fig1 = plt.figure(1,figsize=(8,8),dpi=100,facecolor='white')
# plt.ion()
#keyBoardEvent
# class KeyBoardEvent(PyKeyboardEvent):
#     """docstring for KeyBoardEvent"""
#     def __init__(self):
#         PyKeyboardEvent.__init__(self)

#     def tap(self, keycode, character, press):
#         ''' test the keyborad'''
#         # print "q is quit"
#         global is_Save
#         if press and character == ' ':
#             is_Save = not is_Save
#             if is_Save:
#                 print 'begin to save the data'
#             else:
#                 print 'stop saving'

global frame
point_cloud_pub = rospy.Publisher('points', PointCloud2, queue_size=100)
parser = argparse.ArgumentParser()
parser.add_argument('-s','--sequence', type=str, default='00', help='input sequence number')
opt = parser.parse_args()
#baserdir = '/home/gallop/disk/data/scandata/'
# baserdir = '/home/ysk/catkin_ws/laser_pic_code/pic/'
baserdir = '/home/ysk/database/test_data/'
scandir = 'scan'
sequence = opt.sequence
dirName = os.path.join(baserdir,sequence)

def quater_to_matrix(quater):
    q0,q1,q2,q3 = quater
    q02,q12,q22,q32 = quater * quater
    q0q1=q0*q1;q0q2=q0*q2;q0q3=q0*q3
    q1q2=q1*q2;q1q3=q1*q3;q2q3=q2*q3 
    rotation = np.array([
        1-2*(q22+q32), 2*(q1q2-q0q3), 2*(q1q3+q0q2),
        2*(q1q2+q0q3), 1-2*(q12+q32), 2*(q2q3-q0q1),
        2*(q1q3-q0q2), 2*(q2q3+q0q1), 1-2*(q12+q22)
        ]).reshape(3,3)
    return rotation

def saveScan(data,fname = '0.bin'):
    thePath = os.path.join(dirName,scandir,fname)
    data.tofile(thePath)
    # plt.figure(1)
    # plt.clf() 
    # plt.axis('equal')
    # plt.axis('off')
    # plt.xlim([-15,15])
    # plt.ylim([-15,15])
    # plt.plot(data[:,0], data[:,1],'b')
    # plt.draw();
    # fig1.show()
        # saveD = data[:,:2]
        # saveD.tofile(thePath)
        # print 'scan path',thePath

def saveLaser(data,fname = '0.bin'):
    data = data.reshape(-1)
    # print data.shape
    thePath = os.path.join(dirName,'scan',fname)
    dataS = str(data.tolist()).strip('[').strip(']').replace(',',' ')
    with open(thePath,'ab') as f:
        f.write(dataS + '\n')

def saveImage(data, fname='0.png'):  #data = laserscan
    # saveScan(data,fname[:-4]+'.bin')
    thePath = os.path.join(dirName,'picture',fname)
    # plt.figure(0)
    plt.figure(0,figsize=(4.48,4.48))
    plt.clf() 
    plt.axis('equal')
    plt.axis('off')
    plt.xlim([-30,30])
    plt.ylim([-30,30])
    #plt.plot(data[:,0], data[:,1],'k')
    plt.scatter(data[:,0], data[:,1],c="k",marker=".")
    # plt.draw();
    plt.savefig(thePath)
    plt.cla()
    # print 'image path',thePath

def saveOdom(data,fname = 'poses.txt'):
    dataS = str(data.tolist()).strip('[').strip(']')
    thePath = os.path.join(dirName, fname)
    # print thePath
    with open(thePath,'ab') as f:
        f.write(dataS + '\n')
# def save2image():

def saveTimestamps(data,fname='times.txt'):
    thePath = os.path.join(dirName, fname)
    # print thePath
    with open(thePath,'ab') as f:
        f.write(str(data) + '\n')


def callback(scan,amcl_pose):
    if scan.header.stamp.secs == amcl_pose.header.stamp.secs:
        if len(scan.ranges) <= 10:
            return
        global is_Save
        if is_Save:
            qn = [amcl_pose.pose.pose.orientation.x, amcl_pose.pose.pose.orientation.y,
                  amcl_pose.pose.pose.orientation.z, amcl_pose.pose.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(qn)
            stamp = scan.header.stamp
            #handle the data of scan
            global frame

            global dDelta
            global lastDistance
            points_per_scan=0
            is_first_flag = True
            pointCloud = []
            if is_first_flag == True:
                #print data.angle_min, data.angle_max
                points_per_scan = (scan.angle_max - scan.angle_min) / scan.angle_increment
                is_first_flag = False
            for i in range(int(points_per_scan)+1):
                if i == 0:
                    dDelta = 0
                    if scan.ranges[i] > 0.1 and scan.ranges[i] <30:
                        lastDistance = scan.ranges[i]
                    else:
                        lastDistance = 0
                if math.isinf(scan.ranges[i]) or math.isnan(scan.ranges[i]):
                    ranges = lastDistance #+ dDelta
                else:
                    ranges = scan.ranges[i]
                # if ranges < 0.1:
                #     pointCloud.append([0,0,0])
                #     continue
                theta = scan.angle_min + i * scan.angle_increment - math.pi * 0.5
                x = ranges * math.cos(theta)
                y = ranges * math.sin(theta)
                z = 0.0
                pointCloud.append([x,y,z])
                # dDelta = scan.ranges[i] - lastDistance
                lastDistance = ranges
                # if scan.ranges[i] > 0.05 and scan.ranges[i] <30:
                #     theta =  scan.angle_min + i * scan.angle_increment -math.pi * 0.5
                #     x = scan.ranges[i] * math.cos(theta)
                #     y = scan.ranges[i] * math.sin(theta)
                #     z = 0.0
                #     pointCloud.append([x,y,z])
                # else:
                #     pointCloud.append([0,0,0])

            if len(pointCloud) < 10:
                return
            saveData = np.array(pointCloud)
            #print saveData.shape
            scanFname = str(frame).zfill(5)+'.bin'
            imageFname = str(frame).zfill(5)+'.jpg'

            # saveOdom(transform)
            saveScan(saveData, scanFname)
            # saveLaser(saveData, scanFname)
            saveImage(saveData,imageFname)
            # saveTimestamps(stamp.to_sec())
            pointCloud = []
            saveData = np.array([])
            thePath = os.path.join(dirName, "label.txt")
            label = str(amcl_pose.pose.pose.position.x) + ";" + str(amcl_pose.pose.pose.position.y) + ";" + str(yaw*180/math.pi)
            # label = str(amcl_pose.pose.pose.position.x) + ";" + str(amcl_pose.pose.pose.position.y) +  ";" + \
            #         str(amcl_pose.pose.pose.orientation.z) + ";" + str(amcl_pose.pose.pose.orientation.w)
            with open(thePath, 'ab') as f:
                f.write(str(frame).zfill(5) + '.jpg' + ";" + label + '\n')
            print(frame)
            frame += 1

def main():
    rospy.init_node('scan_listener', anonymous=True)
    print "scan_listener"
    time.sleep(2)
    global frame
    frame = 0
    
    if not os.path.isdir(dirName):
        dataDir = os.path.join(dirName,scandir)
        os.makedirs(dataDir)
        imageDir = os.path.join(dirName,'picture')
        os.makedirs(imageDir)
        # laserDir = os.path.join(dirName,'laser')
        # os.makedirs(laserDir)
        print dirName
    
    # scan_sub = rospy.Subscriber("/scan_test", LaserScan, scanCallback)
    # odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    scan_sub = message_filters.Subscriber("/scan_test", LaserScan)
    pose_sub = message_filters.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
    tfListener = tf.TransformListener()

    ts = message_filters.ApproximateTimeSynchronizer([scan_sub, pose_sub], 10, 0.01)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    global is_Save
    is_Save = True
    # k = KeyBoardEvent()
    # t = threading.Thread(target=k.run, name='keyThread')
    # t.setDaemon(True)
    # try:
    #     t.start() 
    # except KeyboardInterrupt:
    #     print "quit"
    # print 'next is main!!'
    main()
    # t1 = threading.Thread(target=rospy.spin, name='rospy')
    # t1.setDaemon(True)
    # try:
    #     t1.start() 
    # except KeyboardInterrupt:
    #     print "quit"
    
    # plt.ioff()
    # plt.show()

    
