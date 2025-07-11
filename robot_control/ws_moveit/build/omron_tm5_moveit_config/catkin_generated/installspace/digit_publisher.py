#this code is based on what the pytouch library offers with some tweaks and changes using openCV to adapt our sensors to it

#This is a publisher node to integrate the digit sensors in ROS 
# it will publish yhe images from the 2 sensors to image topics in ROS  /LeftFingerCamera & /RightFingerCamera
# it will also publish the contact area data to /FingerData in an array format as follows [Area1, Diff1, Area2, Diff2]
#Area1 is the contact area from sensor1 and Diff1 is the difference between 2 consecutive detected areas

#the sensor_id_1 and sensor_id_2 are used to declare the 2 sensors to be used.
  
#!/usr/bin/env python
import rospy
import time
import math
import actionlib
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

from digit_interface import Digit
from pytouch.handlers import ImageHandler


sensor_id_1 = "D00003"
sensor_id_2 = "D00001"

def get_image_from_sensor(sensor_id):

    d = Digit(sensor_id) # Unique serial number
    d.connect()

    frame = d.get_frame()
    d.disconnect()
    return  frame


def differ(target, base):  #this function is used to detect the difference between 2 images (from pytouch library)
    diff = (target * 1.0 - base) / 255.0 + 0.5
    diff[diff < 0.5] = (diff[diff < 0.5] - 0.5) * 0.7 + 0.5
    diff_abs = np.mean(np.abs(diff - 0.5), axis=-1)
    return diff_abs

def smooth(target): #this function is used to smooth the detected difference between 2 images (from pytouch library)
    kernel = np.ones((64, 64), np.float32)
    kernel /= kernel.sum()
    diff_blur1 = cv2.filter2D(target, -1, kernel)
    return diff_blur1
    
def find_contours(target):  #this function is used to find contours on the smoothed difference image (from pytouch library) 
    mask = ((np.abs(target) > 0.04) * 255).astype(np.uint8)
    kernel = np.ones((16, 16), np.uint8)
    mask = cv2.erode(mask, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    sorted_contours= sorted(contours, key=cv2.contourArea, reverse= True) #this line was added to get the largest contour in the index 0 of the list.
    return sorted_contours


def Area(base,frame):   # this function uses the previous functions to find the largest detected contour and draw the largest circle inside it
                        # calculates and returs the area of this circle and draw it on the frame

	
	diff = differ(frame,base)
	diff2 = smooth(diff) # compute difference and smooth it
	cnt= find_contours(diff2) # find contours
	if (len(cnt) == 0 ):
		area = 0
		frame_copy = base
		return frame_copy, area
	else:
		frame_copy = frame.copy()
		cv2.drawContours(image=frame_copy, contours=cnt, contourIdx=0, color=(0, 0, 255), thickness=1, lineType=cv2.LINE_AA)
		#create blank mask
		mask = np.zeros(frame.shape[:2], dtype="uint8")
		cv2.drawContours(mask, cnt, 0, 255, -1)
		dist = cv2.distanceTransform(mask, cv2.DIST_L2, 0)
		NULL,max_val,NULL,max_indx=cv2.minMaxLoc(dist)
		(x,y),radius = max_indx, int(max_val)
		#draw circle on original image
		cv2.circle(frame_copy, (x,y), radius, (255,0,0), 1)
		area = (radius**2)*math.pi
		return frame_copy, area
	
		
def tactile_publisher():

#this function reads the data from the sensors applies the function and publishes the data.
		
		rospy.init_node('Franka_tactile_publisher')
		LeftFingerCamera = rospy.Publisher('LeftFingerCamera', Image, queue_size=1)
		RightFingerCamera = rospy.Publisher('RightFingerCamera', Image, queue_size=1)
		FingerData = rospy.Publisher('FingerData', Float32MultiArray, queue_size=100)

		base1=get_image_from_sensor(sensor_id_1)
		#ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_3_5.png", base1)
		base2=get_image_from_sensor(sensor_id_2)
		#ImageHandler.save("/home/frankacitius/Desktop/test_digit_contact/omron_tests/base_1_5.png", base2)
		i = 0
		a1_list = [0]
		a2_list = [0]
		a1=0
		a2=0
		dif1=0
		dif2=0
		while(True):

					
			frame1 = get_image_from_sensor(sensor_id_1)
			frame2 = get_image_from_sensor(sensor_id_2)
			img1 , a1 = Area(base1,frame1)
			img2 , a2 = Area(base2,frame2)
			
			copy1=img1.copy()			
			cv2.putText(img=copy1, text="Area="+str(round(a1,2))+"px2", org=(10, 310), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
			
			copy2=img2.copy()			
			cv2.putText(img=copy2, text="Area="+str(round(a2,2))+"px2", org=(10, 310), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)

			
			a1_list.append(a1)
			a2_list.append(a2)
			
			dif1 = abs(a1_list[i+1] - a1_list[i])
			dif2 = abs(a2_list[i+1] - a2_list[i])

			i+=1
			
			array = {round(a1), round(dif1), round(a2), round(dif2)} 
 
			
			data_msg = Float32MultiArray()
			data_msg.data = array


			left_finger = CvBridge().cv2_to_imgmsg(copy1, "bgr8")
			right_finger = CvBridge().cv2_to_imgmsg(copy2 , "bgr8")
			
			
			LeftFingerCamera.publish(left_finger)
			RightFingerCamera.publish(right_finger)
			#print("a1="+str(a1))
			#print("a2="+str(a2))
			#print("dif1="+str(dif1))
			#print("dif2="+str(dif2))			
			FingerData.publish(data_msg)




		
		rospy.spin()
    

if __name__ == '__main__':
    tactile_publisher()


