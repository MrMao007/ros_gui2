#!/usr/bin/env python
#coding=utf-8

import rospy
import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import Obstacles
from custom_msgs.msg import Form
from geometry_msgs.msg import Point

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
    for i in range(len(dr)):
        wall = []
        angle1 = dr[i][1] - theta
        x0 = math.cos(angle1) * z0
        y0 = - math.sin(angle1) * z0
        vec1 = np.array([x0, y0])
        angle2 = dr[i][1] + theta
        x1 = math.cos(angle2) * z0
        y1 = - math.sin(angle2) * z0
        vec2 = np.array([x1, y1])
        p0 = point_converter(dr[i][0] + vec1)
        wall.append(p0)
        p1 = point_converter(dr[i][0] - vec2)
        wall.append(p1)
        p2 = point_converter(dr[i][0] - vec1)
        wall.append(p2)
        p3 = point_converter(dr[i][0] + vec2)
        wall.append(p3)
        line_to_draw.append(wall)
    return(line_to_draw)

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

def talker(door_msg):
    pub = rospy.Publisher('/Vobstacls',Obstacles, queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        walls = Obstacles()
        for lines in door_msg:
            point1 = Point()
            Form1 = Form()
            point1.x = lines[0][0]
            point1.y = lines[0][1]
            point1.z = 0.0
            Form1.form.append(point1)
            point2 = Point()
            point2.x = lines[1][0]
            point2.y = lines[1][1]
            point2.z = 0.0
            Form1.form.append(point2)
            walls.list.append(Form1)
            point3 = Point()
            Form2 = Form()
            point3.x = lines[2][0]
            point3.y = lines[2][1]
            point3.z = 0.0
            Form2.form.append(point3)
            point4 = Point()
            point4.x = lines[3][0]
            point4.y = lines[3][1]
            point4.z = 0.0
            Form2.form.append(point4)
            walls.list.append(Form2)
            circle_dirx = lines[1][0]-lines[2][0]
            circle_diry = lines[1][1]-lines[2][1]
            lenth = math.sqrt(pow(circle_dirx,2)+pow(circle_diry,2))
            circle_dirx = circle_dirx/lenth
            circle_diry = circle_diry/lenth
            circle = Point()
            Form_circle = Form()
            r = 0.3
            circle.x = point1.x + r*circle_dirx
            circle.y = point1.y + r*circle_diry
            circle.z = r
            Form_circle.form.append(circle) 
            walls.list.append(Form_circle)
        pub.publish(walls)
        rate.sleep()

def point_converter(point):
    origin = [-63.053811, -27.172959]
    map_point = []
    map_point.append(point[0]*0.05+origin[0])
    map_point.append((1043 - point[1])*0.05+origin[1])
    return map_point





if __name__ == '__main__':
    img1 = cv2.imread('/home/liu/catkin_ws/src/vwalls/map/medical.jpg')
    door = findcenter(img1)
    wall = door_point(door, 30, 42)
    talker(wall)

