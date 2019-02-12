#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2, LaserScan, Image
import laser_geometry.laser_geometry as lg 
import math
import numpy as np
import cv2
import sys
from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError

def rad_to_degrees(radian):
    return (radian/np.pi) * 180
    
def degrees_to_rad(degrees):
    return (degrees/180) * np.pi

#initialize node and laser projection object
rospy.init_node("uuv_sonar_to_pointcloud")
laser_project = lg.LaserProjection()

#get home directory of machine from UNIX-based "~" sign
home = expanduser("~")

#Configuration of sonar used for simulation. Check uuv_sensor_ros_plugin's sonar_snippet.xacro for details
sonar_name = "blueview_p900"
update_rate = "15"
beam_samples = 512
fov = 1.5708
fov_degrees = rad_to_degrees(fov) #(fov/np.pi) * 180
range_min = 1.0
range_max = 100.0
range_stddev = 0.027 

#image settings
Im_height = 400
Im_width = 600
sonar_origin = (Im_width/2,0) #starts from base/side of image

#prepare cvBridge and image publisher
cvbridge = CvBridge()
rosimg_pub = rospy.Publisher("sonar_image", Image)



# Converts laser data to sonar image. 
# Based on previous work done by DeMarco in paper: https://ieeexplore.ieee.org/document/7404349
def laser_sonar_cvt(laser_msg):

    #convert laser_msgs into pc2 type messages first
    pc2_msg = laser_project.projectLaser(laser_msg)

    #create point generator from the received msg
    point_generator = pc2.read_points(pc2_msg)

    #prepare an empty numpy array to store cartesian values
    point_cartesian = np.zeros((beam_samples,2),dtype=float)
    

    '''
    # note: each point from generator is a tuple that contains the following format:
    # (point) = (x,y,z,intensity,index), with indexing that starts from 0
    '''
    for point in point_generator:
        point_cartesian[point[4]][0] = point[0]
        point_cartesian[point[4]][1] = point[1]
    
    '''
    #TODO: need to calculate intensity of ray reflections from each material. 
    ray_length = np.zeros((beam_samples,1), dtype=float)
    ray_length = np.sqrt(point_cartesian[:,0]**2 + point_cartesian[:,1]**2)
    '''

    #convert cartesian points into image coordinates. 
    # image_coor[:,0] ~ x-coordinate
    # image_coor[:,1] ~ y-coordinate
    image_coor = np.zeros((beam_samples,2), dtype=float)
    image_coor[:,0] = float(Im_width)/2 - ((point_cartesian[:,1]/(math.sin(fov) * range_max)) * math.sin(fov/2) * float(Im_height))
    image_coor[:,1] = point_cartesian[:,0] * float(Im_height) / range_max
    '''
    np.savetxt(home + "/pcl_cartesian.csv", point_cartesian, fmt='%.5f', delimiter=",")
    np.savetxt(home + "/image_coor.csv", image_coor, fmt='%.5f', delimiter=",")
    '''
    cv_img = np.zeros((Im_height,Im_width), dtype=np.uint8)

    snr_start = (180-fov_degrees)/2
    snr_start_rad = degrees_to_rad(snr_start)
    snr_end = snr_start + fov_degrees
    snr_end_rad = degrees_to_rad(snr_end)
    print(snr_start,snr_end)

    cv2.ellipse(cv_img, sonar_origin, (Im_height, Im_height), 0, snr_start, snr_end, 50, -1, 0)

    #map all beams onto image. Note that sonar image is still inverted
    for i in range(0,beam_samples):
        cv2.circle(cv_img,(int(image_coor[i][0]), int(image_coor[i][1])), 1, 200, -1, 8, 0)

    #adding Gaussian noise
    cv_img = cv_img + 50*np.random.normal(0, 0.4,cv_img.shape).astype(np.uint8)

    #rotate image by 180 degrees to get sonar image in correct orientation
    Rot_M = cv2.getRotationMatrix2D((Im_width/2, Im_height/2), 180, 1)
    dst = cv2.warpAffine(cv_img, Rot_M, (Im_width, Im_height))
    dst = cv2.applyColorMap(dst, cv2.COLORMAP_HOT)

    #rotate image by 180 degrees to get sonar image in correct orientation
    Rot_M = cv2.getRotationMatrix2D((Im_width/2, Im_height/2), 180, 1)
    dst = cv2.warpAffine(cv_img, Rot_M, (Im_width, Im_height))
    dst = cv2.applyColorMap(dst, cv2.COLORMAP_HOT)

    #transform from CV2 image to Ros image messages, and publish
    rosimg_pub.publish(cvbridge.cv2_to_imgmsg(dst, "bgr8"))

rospy.Subscriber("rexrov/sonar", LaserScan, laser_sonar_cvt, queue_size=1)
rospy.spin()



