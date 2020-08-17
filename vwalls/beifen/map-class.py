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

from dynamic_reconfigure.server import Server
from vwalls.cfg import widthConfig



#需要配置参数：self.map,self.origin,self,reslution
#需要调整的参数：self.maxwidth_times(最大墙宽，门宽的倍数),self.minwidth,self.length,self.r
class Robot:
    def __init__(self):
        rospy.init_node('door_pass',anonymous=True)
        pub = rospy.Publisher('/Vobstacls',Obstacles, queue_size=10)
        srv = Server(widthConfig,self.callback)
        self.map = cv2.imread('/home/liu/catkin_ws/src/vwalls/map/4.jpg')
        self.origin = [-7.885178, -2.743137]
        self.reslution = 0.025
        self.maxwidth = 1
        self.minwidth = 0.5
        self.length = 0.8
        self.r = 0.2
        self.door = self.door_msg()
        self.in_flag = 0
        rate = rospy.Rate(100) # 10hz
        self.tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            print(self.r)
            rate.sleep()
            if self.get_pos():
                self.pose,self.th = self.get_pos() 
                self.door_to_pass = self.which_door_to_pass()
                messages = self.dynamic()
                pub.publish(messages)
            try:
                self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass
            #rate.sleep()
            #print(self.get_pos())
            #if self.door_to_pass: 
                #print(self.door_to_pass)
                #messages = self.dynamic()
                #pub.publish(messages)
            #rate.sleep()
            #try:
            #    self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
            #except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            #    return

    def callback(self,config,level):
        rospy.loginfo("""参数更新:最大墙宽：{max_width}, 最小墙宽：{min_width}, 墙长度：{length}, 转弯半径：{r}""".format(**config))
        self.maxwidth = config['max_width_times']
        self.minwidth = config['min_width']
        self.length = config['length']
        self.r = config['r']
        return config


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
        pose = [x,y]
        return pose, th


    #地图坐标转全局坐标
    def point_converter(self,point):
        map_point = []
        map_point.append(point[0]*self.reslution+self.origin[0])
        map_point.append((self.map.shape[0] - point[1])*self.reslution+self.origin[1])
        return map_point

    #由地图提取所有门信息,返回格式[[门中点1]，门宽，垂线与x轴夹角]×n
    def door_msg(self):
        search_map = self.map
        gray = cv2.cvtColor(search_map, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        _,contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        door = []
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if (area < 1000):
                rect = cv2.minAreaRect(cnt)
                doorsp = []
                doorsp.append(self.point_converter(rect[0]))
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                height_v = box[0] - box[1]
                width_v = box[2] - box[1]
                p0 = np.array([1, 0])
                height = math.hypot(height_v[0], height_v[1])
                width = math.hypot(width_v[0], width_v[1])
                if (height > width):
                    doorsp.append(height*self.reslution)
                    if (box[2][1] > box[1][1]):
                        width_v = box[1] - box[2]
                    cos = width_v.dot(p0)/width
                else:
                    doorsp.append(width*self.reslution) 
                    if (box[0][1] > box[1][1]):
                        height_v = box[1] - box[0]
                    cos = height_v.dot(p0)/height
                angle = np.arccos(cos)
                a = angle*180/np.pi
                doorsp.append(angle)
                door.append(doorsp)
        #imag = cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
        return door
   
    #结合定位确定需要过的门，返回：[对应门信息，同上]   
    def which_door_to_pass(self):
        x = self.pose[0]
        y = self.pose[1]
        mindis = 1000000
        doorway = []
        for each_door in self.door:
            doorcenter = each_door[0]
            th = each_door[2] 
            dis = pow((x-doorcenter[0]),2)+pow((y-doorcenter[1]),2)
            if dis < mindis:
                mindis = dis
                #x_rotate = math.cos(th)*(x-doorcenter[0])+math.sin(th)*(y-doorcenter[1]) - doorcenter[0]
                #y_rotate = math.cos(th)*(y-doorcenter[1])-math.sin(th)*(x-doorcenter[0]) - doorcenter[1]
                #if x_rotate >=0 and y_rotate >=0:
                #    in_point = 0
                #if x_rotate >=0 and y_rotate <0:
                #    in_point = 3
                #if x_rotate <=0 and y_rotate <=0:
                #    in_point = 2
                #if x_rotate <0 and y_rotate >0:
                #    in_point = 1
                door_to_pass = each_door
        doorway.append(door_to_pass)
        #doorway.append(in_point)
        return doorway

         
    #没有定位时测试     
    def which_door_to_pass_test(self):
        (x,y)=(0.0)
        mindis = 1000000
        doorway = []
        for each_door in self.door:
            doorcenter = each_door[0]
            th = each_door[2] 
            dis = pow((x-doorcenter[0]),2)+pow((y-doorcenter[1]),2)
            if dis < mindis:
                mindis = dis
                #x_rotate = math.cos(th)*(x-doorcenter[0])+math.sin(th)*(y-doorcenter[1])
                #y_rotate = math.cos(th)*(y-doorcenter[1])-math.sin(th)*(x-doorcenter[0])
                #if x_rotate >=0 and y_rotate >=0:
                #    in_point = 0
                #if x_rotate >=0 and y_rotate <0:
                #    in_point = 3
                #if x_rotate <=0 and y_rotate <=0:
                #    in_point = 2
                #if x_rotate <0 and y_rotate >0:
                #    in_point = 1
                door_to_pass = each_door
        doorway.append(door_to_pass)
        #print(doorway)
        return doorway
   
    #根据前述函数和指定参数生成门点
    def door_boundary_point(self,l,w,r):
        z = math.sqrt(l*l+w*w)
        z0 = z/2
        theta = np.arccos(w/z)
        door_bpoint = []
        walls = Obstacles()
        if self.door_to_pass:
            #四个顶点
            angle1 = self.door_to_pass[0][2] - theta
            x0 = math.cos(angle1) * z0
            y0 = math.sin(angle1) * z0
            vec1 = np.array([x0, y0])
            angle2 = self.door_to_pass[0][2] + theta
            x1 = math.cos(angle2) * z0
            y1 = math.sin(angle2) * z0
            vec2 = np.array([x1, y1])
            Form0 = Form()
            p0 = self.door_to_pass[0][0] + vec1
            door_bpoint.append(p0)
            point0 = Point()
            point0.x = p0[0]
            point0.y = p0[1]
            point0.z = 0.0
            Form0.form.append(point0)
            p1 = self.door_to_pass[0][0] - vec2
            door_bpoint.append(p1)
            point1 = Point()
            point1.x = p1[0]
            point1.y = p1[1]
            point1.z = 0.0
            Form0.form.append(point1)
            walls.list.append(Form0)
            Form1 = Form()
            p2 = self.door_to_pass[0][0] - vec1
            door_bpoint.append(p2)
            point2 = Point()
            point2.x = p2[0]
            point2.y = p2[1]
            point2.z = 0.0
            Form1.form.append(point2)
            p3 = self.door_to_pass[0][0] + vec2
            door_bpoint.append(p3)
            point3 = Point()
            point3.x = p3[0]
            point3.y = p3[1]
            point3.z = 0.0
            Form1.form.append(point3)
            walls.list.append(Form1)
            #圆弧，判断离哪个点近
            mindis = 100000
            cornerid = 0
            tgid = 0
            for corner in door_bpoint:
                dis = pow((self.pose[0]-corner[0]),2)+pow((self.pose[1]-corner[1]),2)
                if dis < mindis:
                    tgid = cornerid
                    mindis = dis
                cornerid +=1
            print(tgid)

            circle_dir = list(map(lambda x: x[0]-x[1], zip(p2, p1)))
            lenth = math.sqrt(pow(circle_dir[0],2)+pow(circle_dir[1],2))
            circle_dirx = circle_dir[0]/lenth
            circle_diry = circle_dir[1]/lenth
            circle = Point()
            Form_circle = Form()
            if tgid > 1:
                circle.x = door_bpoint[tgid][0] + r*circle_dirx
                circle.y = door_bpoint[tgid][1] + r*circle_diry
                circle.z = r
            else:
                circle.x = door_bpoint[tgid][0] - r*circle_dirx
                circle.y = door_bpoint[tgid][1] - r*circle_diry
                circle.z = r
            Form_circle.form.append(circle) 
            walls.list.append(Form_circle)
        return door_bpoint,walls


    def pass_process(self,pos,door_area):
        cross1 = (door_area[1][0]-door_area[0][0])*(pos[1]-door_area[0][1])-(pos[0]-door_area[0][0])*(door_area[1][1]-door_area[0][1])
        cross2 = (door_area[2][0]-door_area[2][0])*(pos[1]-door_area[2][1])-(pos[0]-door_area[2][0])*(door_area[3][1]-door_area[3][1])
        cross3 = (door_area[2][0]-door_area[1][0])*(pos[1]-door_area[1][1])-(pos[0]-door_area[1][0])*(door_area[2][1]-door_area[1][1])
        cross4 = (door_area[0][0]-door_area[3][0])*(pos[1]-door_area[3][1])-(pos[0]-door_area[3][0])*(door_area[0][1]-door_area[3][1])
        if cross1*cross2 >=0 and cross3*cross4 >=0:
            return 1
        else:
            return 0
    
    #判定当前是否在过门阶段
    def pass_process_test(self,door_area):
        cross1 = (door_area[1][0]-door_area[0][0])*(y-door_area[0][1])-(x-door_area[0][0])*(door_area[1][1]-door_area[0][1])
        cross2 = (door_area[2][0]-door_area[2][0])*(y-door_area[2][1])-(x-door_area[2][0])*(door_area[3][1]-door_area[3][1])
        cross3 = (door_area[2][0]-door_area[1][0])*(y-door_area[1][1])-(x-door_area[1][0])*(door_area[2][1]-door_area[1][1])
        cross4 = (door_area[0][0]-door_area[3][0])*(y-door_area[3][1])-(x-door_area[3][0])*(door_area[0][1]-door_area[3][1])
        if cross1*cross2 >=0 and cross3*cross4 >=0:
             return 1
        else:
             return 0

    def dynamic(self):
        if self.in_flag == 0:
            k,walls = self.door_boundary_point(self.minwidth,self.length,self.r)
        else:
            k,walls = self.door_boundary_point(self.maxwidth,self.length,self.r)
        self.in_flag = self.pass_process(self.pose,k)
        return walls

if __name__ == '__main__':
    r = Robot()

    


