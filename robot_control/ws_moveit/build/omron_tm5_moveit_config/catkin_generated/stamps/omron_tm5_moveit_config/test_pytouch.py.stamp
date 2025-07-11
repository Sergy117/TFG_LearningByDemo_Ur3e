#this code was ude to test the touch_detect provided by pytouch on the omron robot, it also includes the ContactArea task from pytouch for testing purposes

#!/usr/bin/env python
import rospy
import pytouch
import time
import math
import cv2
import actionlib
from PIL import Image
from matplotlib import cm
import numpy as np

from digit_interface import Digit
from pytouch.handlers import ImageHandler
from pytouch.sensors import DigitSensor
from pytouch.tasks import ContactArea
from pytouch.tasks import TouchDetect

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
		
def extract_surface_contact(frame,base,save_name):
		base_img=base
		sample_img = frame


		# initialize with custom configuration of ContactArea task
		contact_area = ContactArea(base=base_img, contour_threshold=0)
		major, minor = contact_area(sample_img)
		
		l_major = math.sqrt((major[1][0] - major[0][0])**2 + (major[1][1] - major[0][1])**2)  /2 
		l_minor = math.sqrt((minor[1][0] - minor[0][0])**2 + (minor[1][1] - minor[0][1])**2)  /2 
		
		area =  l_major * l_minor * 3.14

		print("Major Axis: {}, minor axis: {}".format(major, minor))
		ImageHandler.save(save_name, sample_img)
		
		return area 
		
		
def touch_detect(img):
	
	
	#PIL_image = Image.fromarray(np.uint8(img)).convert('RGB')

	PIL_image = Image.fromarray(img.astype('uint8'), 'RGB')

    # initialize with custom configuration of TouchDetect task
	touch_detect = TouchDetect(DigitSensor, zoo_model="touchdetect_resnet",model_path="/home/frankacitius/Downloads/touchdetect_resnet_DigitSensor.pth")

	is_touching, certainty = touch_detect(PIL_image)
	print(f"Is touching? {is_touching}, {certainty}")
	
	return is_touching, certainty 
		
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
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_3_5.png", base1)
		base2=get_image_from_sensor("D00001")
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_1_5.png", base2)
		
		print("bases are taken")
		
		command=GripperCommand
		command.position=command_init.position
		command.max_effort=5
		goal= GripperCommandGoal
		
		c1=c2=0
		s2=s1=0
		while( (s1 == 0  or c1 < 0.99) or (s2 == 0 or c2<0.99)) :
			frame1 = get_image_from_sensor("D00003")
			frame2 = get_image_from_sensor("D00001")
	
			s1, c1 = touch_detect(frame1)
			s2, c2 = touch_detect(frame2)
			
			command.position=command.position - 0.001
		  
			goal.command=command
			client.send_goal(goal)
			
			
		print("object grasped")
		frame1 = get_image_from_sensor("D00003")
		frame2 = get_image_from_sensor("D00001")
		
		
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/frame_3_6.png", frame1)
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/frame_1_6.png", frame2)
		
		time.sleep(1)
		
		
		contact_area1 = ContactArea(base=base1, contour_threshold=100)
		major1, minor1 = contact_area1(frame1)
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/touch_detect_1_6.png", frame1)
		
		print("frame taken")
		time.sleep(1)
		
		contact_area2 = ContactArea(base=base2, contour_threshold=100)
		major2, minor2 = contact_area2(frame2)
		ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/touch_detect_3_6.png", frame2)	
		
		print("frame1 taken")
		

		

		cv2.waitKey(0)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
    

if __name__ == '__main__':
    tactile_sevoing()
