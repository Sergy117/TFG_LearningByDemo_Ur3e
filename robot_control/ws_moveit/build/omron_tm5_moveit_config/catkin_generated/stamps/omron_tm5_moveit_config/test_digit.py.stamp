#this code tests the tactile servoing without moving the robot
#!/usr/bin/env python
import rospy
import pytouch
import time
import math
import actionlib
import cv2
from PIL import Image
from matplotlib import cm
import numpy as np


from digit_interface import Digit
from pytouch.handlers import ImageHandler
from pytouch.sensors import DigitSensor

from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandActionGoal
from control_msgs.msg import GripperCommandGoal
from control_msgs.msg import GripperCommand

def get_image_from_sensor(sensor_id):

    d = Digit(sensor_id) # Unique serial number
    d.connect()

    frame = d.get_frame()
    d.disconnect()
    return  frame


def differ(target, base):
    diff = (target * 1.0 - base) / 255.0 + 0.5
    diff[diff < 0.5] = (diff[diff < 0.5] - 0.5) * 0.7 + 0.5
    diff_abs = np.mean(np.abs(diff - 0.5), axis=-1)
    return diff_abs

def smooth(target):
    kernel = np.ones((64, 64), np.float32)
    kernel /= kernel.sum()
    diff_blur1 = cv2.filter2D(target, -1, kernel)
    return diff_blur1
def find_contours(target):
    mask = ((np.abs(target) > 0.04) * 255).astype(np.uint8)
    kernel = np.ones((16, 16), np.uint8)
    mask = cv2.erode(mask, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def Area(base,frame):

	# compute difference
	diff = differ(frame,base)
	diff2 = smooth(diff)
	cnt = find_contours(diff2)
	
	if (len(cnt) == 0 ):
		area = 0
		frame_copy = 0
		return frame_copy, area
	else:
		frame_copy = frame.copy()
		cv2.drawContours(image=frame_copy, contours=cnt, contourIdx=-1, color=(0, 0, 255), thickness=1, lineType=cv2.LINE_AA)
		#create blank mask
		mask = np.zeros(frame.shape[:2], dtype="uint8")
		cv2.drawContours(mask, cnt, -1, 255, -1)
		dist = cv2.distanceTransform(mask, cv2.DIST_L2, 0)
		NULL,max_val,NULL,max_indx=cv2.minMaxLoc(dist)
		(x,y),radius = max_indx, int(max_val)
		#draw circle on original image
		cv2.circle(frame_copy, (x,y), radius, (255,0,0), 1)
		area = (radius**2)*math.pi
		return frame_copy, area
	
		
def tactile_sevoing():
		
		rospy.init_node('Omron_tactile_servoing')
		
		client = actionlib.SimpleActionClient('/onrobot/gripper_action', GripperCommandAction)
		client.wait_for_server()

		
		command_init=GripperCommand
		command_init.position=0.07
		command_init.max_effort=5
		
		goal_init= GripperCommandGoal
		goal_init.command=command_init
		client.send_goal(goal_init)
		client.wait_for_result()
		
		print("Start")
		
		base1=get_image_from_sensor("D00003")
		#ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_3_5.png", base1)
		base2=get_image_from_sensor("D00001")
		#ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_1_5.png", base2)
		
		print("bases are taken")
		
		command=GripperCommand
		command.position=command_init.position
		command.max_effort=5
		goal= GripperCommandGoal
		i = 0
		a1_list = [0]
		a2_list = [0]
		a1=0
		a2=0
		dif1=0
		dif2=0
		while((a1==0 or a2==0) or (dif1>1500 or dif2>1500)):
			command.position=command.position - 0.0005
			goal.command=command
			client.send_goal(goal)	
					
			frame1 = get_image_from_sensor("D00003")
			frame2 = get_image_from_sensor("D00001")
			img1 , a1 = Area(base1,frame1)
			img2 , a2 = Area(base2,frame2)
			#print("a1="+str(a1))
			#print("a2="+str(a2))
			
			a1_list.append(a1)
			a2_list.append(a2)
			
			dif1 = abs(a1_list[i+1] - a1_list[i])
			dif2 = abs(a2_list[i+1] - a2_list[i])
			#print("dif1="+str(dif1))
			#print("dif2="+str(dif2))
			i+=1




			
			
		print("object grasped")
		print("object width = "+str(round(command.position*1000,2))+" mm")
		#print(a1_list)
		#print(a2_list)
		
		
		#ImageHandler.save("/home/frankacitius/Desktop/test_openCV/opencv/test_1-1.png", img2)
		#ImageHandler.save("/home/frankacitius/Desktop/test_openCV/opencv/test_3-1.png", img1)
		
		cv2.imshow("sensor1",img1)
		cv2.imshow("sensor2",img2)

		cv2.waitKey(0)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
    

if __name__ == '__main__':
    tactile_sevoing()

















