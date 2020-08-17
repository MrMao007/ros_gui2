#!/usr/bin/env python
#coding=utf-8

import rospy
import cv2
import numpy as np
import math
from tf_conversions import transformations
import tf
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import Obstacles
from custom_msgs.msg import Form
from geometry_msgs.msg import Point

class Robot:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)
        #print euler[2] / pi * 180

        x = trans[0]
        y = trans[1]
        th = euler[2] / math.pi * 180
        return (x, y, th)

def findcenter(image):
    img = image    
    sp = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    _,contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    door = []
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        if (area < 1000):
            rect = cv2.minAreaRect(cnt)
            doorsp = []
            doorsp.append(rect[0])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            height_v = box[0] - box[1]
            width_v = box[2] - box[1]
            p0 = np.array([1, 0])
            height = math.hypot(height_v[0], height_v[1])
            width = math.hypot(width_v[0], width_v[1])
            if (height > width):
                if (box[2][1] > box[1][1]):
                    width_v = box[1] - box[2]
                cos = width_v.dot(p0)/width
            else:
                if (box[0][1] > box[1][1]):
                   height_v = box[1] - box[0]
                cos = height_v.dot(p0)/height
            angle = np.arccos(cos)
            a = angle*180/np.pi
            doorsp.append(angle)
            door.append(doorsp)

    imag = cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
    return door

def door_point(door,x,y):
    path_w = x
    path_l = y
    z = math.sqrt(x*x+y*y)
    z0 = z / 2
    theta = np.arccos(path_l / z)
    dr = door
    line_to_draw = []
    walls = Obstacles()
    for i in range(len(dr)):
        angle1 = dr[i][1] - theta
        x0 = math.cos(angle1) * z0
        y0 = - math.sin(angle1) * z0
        vec1 = np.array([x0, y0])
        angle2 = dr[i][1] + theta
        x1 = math.cos(angle2) * z0
        y1 = - math.sin(angle2) * z0
        vec2 = np.array([x1, y1])
        Form0 = Form()
        p0 = point_converter(dr[i][0] + vec1)
        point0 = Point()
        point0.x = p0[0]
        point0.y = p0[1]
        point0.z = 0.0
        Form0.form.append(point0)
        p1 = point_converter(dr[i][0] - vec2)
        point1 = Point()
        point1.x = p1[0]
        point1.y = p1[1]
        point1.z = 0.0
        Form0.form.append(point1)
        walls.list.append(Form0)
        Form1 = Form()
        p2 = point_converter(dr[i][0] - vec1)
        point2 = Point()
        point2.x = p2[0]
        point2.y = p2[1]
        point2.z = 0.0
        Form1.form.append(point2)
        p3 = point_converter(dr[i][0] + vec2)
        point3 = Point()
        point3.x = p3[0]
        point3.y = p3[1]
        point3.z = 0.0
        Form1.form.append(point3)
        walls.list.append(Form1)
    return(walls)

def draw_line(image,line):
    vwalls = line
    img = image
    for i in vwalls:
        ptstart = tuple(np.array(i[0], dtype=int))
        ptend = tuple(np.array(i[1], dtype=int))
        img = cv2.line(img, ptstart, ptend, (0, 0, 255), 2, 4)
        ptstart = tuple(np.array(i[2], dtype=int))
        ptend = tuple(np.array(i[3], dtype=int))
        img = cv2.line(img, ptstart, ptend, (0, 0, 255), 2, 4)
    return(img)

def talker(door):
    pub = rospy.Publisher('/Vobstacls',Obstacles, queue_size=10)
    rospy.init_node('talker',anonymous=True)
    robot = Robot()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if robot.get_pos():
            x,y,z = robot.get_pos()  
            #dist1 = math.sqrt(pow((x-point1.x),2)+pow((y-point1.y),2))
            #dist2 = math.sqrt(pow((x-point2.x),2)+pow((y-point2.y),2))
            #dist3 = math.sqrt(pow((x-point3.x),2)+pow((y-point3.y),2))
            #dist4 = math.sqrt(pow((x-point4.x),2)+pow((y-point4.y),2))
            #if dist1 < dist2:
            print(x,y)  
            if y < 12.5:
                print(y)
                wall = door_point(door, 40, 42)
            else: 
                wall = door_point(door, 25, 42)   
            pub.publish(wall)
        rate.sleep()

def point_converter(point):
    origin = [-7.885178, -2.743137]
    map_point = []
    map_point.append(point[0]*0.025+origin[0])
    map_point.append((1299 - point[1])*0.025+origin[1])
    return map_point

if __name__ == '__main__':
    img1 = cv2.imread('/home/liu/catkin_ws/src/vwalls/map/4.jpg')
    me_door = findcenter(img1)
    talker(me_door)

    


