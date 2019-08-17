#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time
import numpy as np

from abstractCcr import *
from geometry_msgs.msg import Twist

class RandomBot(AbstractCcr):
    '''
    AbstractCcr を継承
    update cmd_vel 1Hz

    run randomly
      50% straight
      25% turn left
      25% turn right 
    if bumper hit
       back
    '''

    def find_red_ball(self):
	self.img = cv2.rectangle(self.img, (0,240), (640,480), (0,0,0), -1)
#        color_min = np.array([0,0,100])
#        color_max = np.array([100,100,255])
#       color_mask = cv2.inRange(self.img, color_min, color_max)
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([0,100,150])
        color_max = np.array([50,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
#        cv2.imwrite("/home/koy/catkin_ws/src/burger_war/color_mask.bmp", color_mask)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
#        cv2.imwrite("/home/koy/catkin_ws/src/burger_war/bin.bmp", bin_img)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
#        cv2.imwrite("/home/koy/catkin_ws/src/burger_war/label.bmp", label_img)
#        print("Labeling:", nLabels)
#        for i in range(nLabels - 1):
#            print(data[i])
        if nLabels < 2:
#            print("Labeling: NO: ", nLabels)
            return (-2.0, -2.0)
       
        size_max = 0
        size_max_x = 0
        size_max_y = 0
        size_max_w = 0
        size_max_h = 0
        for i in range(1, nLabels):
            x, y, w, h, size = data[i]
            if size > size_max:
                size_max_x = x
		size_max_y = y
		size_max_w = w
		size_max_h = h
		size_max = size
#                print("Labeling: ", i, size_max_x, size_max_y, size_max_w, size_max_h, size_max)

        hrz = float(size_max_x + size_max_w/2 - 320) / 320.0
        vrt = float(size_max_y + size_max_h/2 - 240) / 240.0
#        print("hrz=", hrz, size_max_x, " vrt=", vrt, size_max_y)

        return (hrz, vrt)


    def strategy(self):
        r = rospy.Rate(100)

        UPDATE_FREQUENCY = 1 # 1sec
        update_time = 0

	img_cntr_prev = 0
	lidar_cntr_prev = 0

        while not rospy.is_shutdown():
#            if self.left_bumper or self.right_bumper:
#                update_time = time.time()
#                rospy.loginfo('bumper hit!!')
#                x = -0.2
#                th = 0

	    if self.lidar_cntr > 0 and self.scan.ranges[0] < 0.2:
                update_time = time.time()
                rospy.loginfo('bumper hit!!')
                x = -0.2
		if self.lidar_cntr % 2 == 0:
                    th = 2
		else:
                    th = -2

	    else:
		hrz = -2.0
		vrt = -2.0

		if self.img_cntr != img_cntr_prev:
		    img_cntr_prev = self.img_cntr
		    hrz, vrt = self.find_red_ball()
		    print(hrz, vrt)

                value = random.randint(1,1000)
                print(value)

                if hrz == 0:
                    x = 0.4
                    th = 0
                    update_time = time.time()
                    print("A")
                elif hrz == -2:
		    if time.time() - update_time > UPDATE_FREQUENCY:
                        update_time = time.time()
                        if value < 500:
                            x = 0.4
                            th = 0
                            print("B")
                        elif value < 750:
                            x = 0
                            th = 2
                            print("C")
                        elif value < 1000:
                            x = 0
                            th = -2
                            print("D")
                        else:
                           x = 0
                           th = 0
                           print("E")
                elif hrz < 0:
                    x = 0.4
                    th = 0.1
                    print("F")
                elif hrz > 0:
                    x = 0.4
                    th = -0.1
                    print("G")
                else:
                    x = 0
                    th = 0
                    print("H")


            twist = Twist()
            twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_ccr')
    bot = RandomBot(use_bumper=True, use_camera=True, camera_preview=True, use_lidar=True)
    bot.strategy()
