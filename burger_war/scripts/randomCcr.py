#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time
import numpy as np
from enum import Enum

from abstractCcr import *
from geometry_msgs.msg import Twist

class BotState(Enum):
    SEARCHING = 1
    APPROACHING = 2
    AIMING = 3
    AIMING_REVERT = 4

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

    def find_specific_color(self, hue_min, hue_max):
#        self.img = cv2.rectangle(self.img, (0,240), (640,480), (0,0,0), -1)
        self.img = cv2.rectangle(self.img, (0,360), (640,480), (0,0,0), -1)
#        color_min = np.array([0,0,100])
#        color_max = np.array([100,100,255])
#       color_mask = cv2.inRange(self.img, color_min, color_max)
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([hue_min,100,150])
        color_max = np.array([hue_max,255,255])
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

        hrz = -2.0
        vrt = -2.0
        img_w, img_h, img_c = self.img.shape
        if size_max > (img_w * img_h) / 10000:
            hrz = float(size_max_x + size_max_w/2 - img_w) / img_w
            vrt = float(size_max_y + size_max_h/2 - img_h) / img_h
#            print("hrz=", hrz, size_max_x, " vrt=", vrt, size_max_y)
            self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 255, 255), 3)

        return (hrz, vrt)


    def find_red_ball(self):
        return self.find_specific_color(0, 50)

    def find_green_marker(self):
        return self.find_specific_color(50, 100)

    def strategy(self):
        r = rospy.Rate(100)

        UPDATE_FREQUENCY = 1 # 1sec
        update_time = 0

	img_cntr_prev = 0
	lidar_cntr_prev = 0

#        self.img_orig = cv2.imread("/home/koy/catkin_ws/src/burger_war/sample.png")
#        self.img_orig = cv2.imread("/home/koy/catkin_ws/src/burger_war/red.png")
#        print(self.img_orig)
#        self.img_cntr = 1

	bot_state = BotState.SEARCHING
	isBumperHit = False
	hrz = -2.0
	vrt = -2.0
	prev_hrz = -2.0
	prev_vrt = -2.0
	prev_valid_vrt = -2.0
        while not rospy.is_shutdown():
	    if self.lidar_cntr != lidar_cntr_prev:
		lidar_cntr_prev = self.lidar_cntr
		if self.scan.ranges[0] < 0.2 or self.scan.ranges[10] < 0.2 or self.scan.ranges[350] < 0.2:
		    isBumperHit = True
#		    rospy.loginfo('bumper hit!!')
		else:
		    isBumperHit = False


	    isImgProcUpdated = False
	    if self.img_cntr != img_cntr_prev:
		img_cntr_prev = self.img_cntr
		isImgProcUpdated = True
		prev_hrz = hrz
		prev_vrt = vrt
		if prev_vrt != -2.0:
		    prev_valid_vrt = prev_vrt
		hrz, vrt = self.find_green_marker()
		logStr = "Green: ", hrz, vrt
		rospy.loginfo(logStr)
		if (hrz == -2.0):
		    hrz, vrt = self.find_red_ball()
#		    print("Red: ", hrz, vrt)
		    logStr = "Red: ", hrz, vrt
		    rospy.loginfo(logStr)
		cv2.imshow("Image window", self.img)
		cv2.waitKey(1)

	    value = random.randint(1,1000)
#	    print(value)

	    if isBumperHit == True:
                update_time = time.time()
#                rospy.loginfo('bumper hit!!')
                x = -0.2
		if value < 500:
                    th = 0.2
		else:
                    th = -0.2
                #print("BUMPER", self.scan.ranges[0], self.scan.ranges[10], self.scan.ranges[350], x, th)
#		print("A", x, th)
		
	    elif bot_state == BotState.SEARCHING:
		if hrz != -2.0:
		    prev_hrz = -2.0
		    prev_vrt = -2.0
		    bot_state = BotState.APPROACHING
		    update_time = 0
#		    print("Bot State changed: SEARCHING -> APPROACHING", hrz)

		elif prev_valid_vrt != -2.0 and prev_valid_vrt < -0.8:
#		    print("Too close??", prev_valid_vrt)
		    prev_valid_vrt = -2.0
		    update_time = time.time()
		    x = -1.0
		    th = 0

		elif time.time() - update_time > UPDATE_FREQUENCY * 2:
		    update_time = time.time()
		    if value < 400:
			x = 0
			th = 2
#			print("B", x, th)
		    elif value < 800:
			x = 0
			th = -2
#			print("C", x, th)
		    else:
			x = 0.4
			th = 0
#			print("D", x, th)

	    elif bot_state == BotState.APPROACHING:
#		print("APPROACHING...", hrz, vrt)

		if hrz == -2.0:
#		    print("Bot State changed: APPROACHING -> SEARCHING", hrz)
		    bot_state = BotState.SEARCHING

		elif vrt < -0.9:
#		    print("Bot State changed: APPROACHING -> AIMING", vrt)
		    bot_state = BotState.AIMING
		    update_time = time.time()

		elif isImgProcUpdated == True and hrz == prev_hrz and vrt == prev_vrt:
		    update_time = time.time()
		    if value < 400:
			x = 0
			th = 0.2
		    elif value < 800:
			x = 0
			th = -0.2
		    else:
			x = 0.2
			th = 0
#		    print("F", x, th)

		elif time.time() - update_time > UPDATE_FREQUENCY:
		    x = 0.5
		    th = hrz * -0.5
#		    print("G", x, th)

	    elif bot_state == BotState.AIMING:
		if time.time() - update_time > UPDATE_FREQUENCY:
		    bot_state = BotState.AIMING_REVERT
		    update_time = time.time()

		else:
		    x = 0.2
		    th = 0.2
#		    print("H", x, th)

#		if hrz == -2.0:
#		    print("Bot State changed: AIMING -> SEARCHING", hrz)
#		    bot_state = BotState.SEARCHING

#		if vrt > -0.7:
#		    print("Bot State changed: AIMING -> APPROACHING", vrt)
#		    bot_state = BotState.APPROACHING
#		    update_time = 0

#		else:
#		    x = 1
#		    th = 0
#		    print("H", x, th)

	    elif bot_state == BotState.AIMING_REVERT:
		if time.time() - update_time > UPDATE_FREQUENCY:
		    bot_state = BotState.APPROACHING

		else:
		    x = 0
		    th = -0.2
#		    print("I", x, th)

	    else:
		print("Impossible Path !!!")
		

            twist = Twist()
            twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_ccr')
    bot = RandomBot(use_bumper=True, use_camera=True, camera_preview=False, use_lidar=True)
    bot.strategy()

