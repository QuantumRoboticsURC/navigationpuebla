#!/usr/bin/env python3

"""Made by:Erika García Sánchez
	A01745158@tec.mx
	erika.mgs@outlook.com
	José Ángel del Ángel
    joseangeldelangel10@gmail.com

Modified (15/12/2022): 
		José Ángel del Ángel and Erika García 16/12/2022 Aruco detection code cleanup 
		José Ángel del Ángel 16/12/2022 Aruco mask with distance added

Code description:
TODO - update code description and notes 
1. Ask the user for the name of a file.
2. If the file exist at a predefined directory it opens it and read it.
   If not, it creates a file with the name.
3. Ask the user the action to execute (write lat & long | write the square).
4. Close the file. 

Notes:
- Validate the user input
- Add an exit option
* Despite the code adds 0.00001 theorically, it's not exact.
"""
import rospy
import pyzed.sl as sl
import numpy as np
import argparse
import time
import cv2
import sys
import math
from std_msgs.msg import String, Int8, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point


class ArucoDetector():
    def __init__(self, aruco_dict = cv2.aruco.DICT_4X4_50):
        rospy.init_node("aruco_detector")

        # ________ aruco atributes initialization ______
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # ________ camera atributes initialization ______
        self.zed_camera = sl.Camera()
        self.zed_init_params = sl.InitParameters()
        self.zed_init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
        self.zed_init_params.camera_resolution = sl.RESOLUTION.HD720
        err = self.zed_camera.open(self.zed_init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)
        rospy.sleep(1.0)
        self.zed_runtime_parameters = sl.RuntimeParameters()
        self.zed_runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # Use STANDARD sensing mode
        #sl.SENSING_MODE.FILL
        self.zed_runtime_parameters.confidence_threshold = 100
        self.zed_runtime_parameters.textureness_confidence_threshold = 100
        self.image_size = self.zed_camera.get_camera_information().camera_resolution
        self.image_size.width = 640
        self.image_size.height = 360

        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.depth_image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat()

        self.imge_ocv = np.zeros((self.image_size.height, self.image_size.width, 3), dtype=np.uint8)
        self.depth_image_zed_ocv = np.zeros((self.image_size.height, self.image_size.width), dtype=np.uint8)
        self.point_cloud_ocv = np.zeros((self.image_size.height, self.image_size.width), dtype=np.uint8)
        self.displayed_image_ocv = np.zeros((self.image_size.height, self.image_size.width, 3), dtype=np.uint8)
        self.arucos_mask = np.zeros((self.image_size.height, self.image_size.width, 3), dtype = np.uint8)
        self.arucos_mask_with_distance = np.zeros((self.image_size.height, self.image_size.width), dtype = np.float64)

        # ________ ros atributes initialization ______        
        self.closest_aruco_position_publisher = rospy.Publisher("/closest_aruco_distance", Point, queue_size = 1)
        self.image_pub = rospy.Publisher("/image_detecting", Image, queue_size = 1)
        self.image_aruco_mask = rospy.Publisher("/image_arucos_mask", Image, queue_size = 1)
        self.image_aruco_mask_distance = rospy.Publisher("arucos_mask_with_distance", Image, queue_size = 1)        
        self.image_point_cloud_ocv = rospy.Publisher("/image_point_cloud_ocv", Image, queue_size = 1)
        #__________ image ______________
        self.curr_signs_image_msg = Image()
        self.curr_signs_image_msg_2 = Image()
        self.curr_signs_image_msg_3 = Image()
        self.curr_signs_image_msg_4 = Image()

        self.square_filter_trh = 20


    def draw_arucos(sel, image, corners):
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
			# loop over the detected ArUCo corners
            for markerCorner in corners:
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
                corners = markerCorner.reshape((4, 2))

                (topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
                image = cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                image = cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                image = cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                image = cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
        return image

    def get_arucos_info_in_image(self, image):
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = self.arucoDetector.detectMarkers(image)    
        return (corners, ids)

    def midpoint_equation(self, p1, p2): # TODO change to 2 equation model 
        return ( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2 )

    def get_aruco_midpoint(self, rectangle_corners):        
        """ function that returns the x,y,z cordinates of the aruco's midpoint """
        # ______________ initializing and formating data that will be used ______________
        self.arucos_mask = np.zeros((self.image_size.height, self.image_size.width, 3), dtype = np.uint8)
        rectangle_corners_for_x_y = rectangle_corners.reshape((4,2))
        rectangle_corners_for_mask = np.int32(rectangle_corners.reshape((1,4,2)))
        # ______________ getting the x,y cordinates of the aruco tag (in pixels) ______________
        x_center, y_center = self.midpoint_equation(rectangle_corners_for_x_y[0,:], rectangle_corners_for_x_y[2,:])
        # ______________ getting the z cordinate of the aruco tag (in point cloud units) ______________
        # step one - we filter the point cloud using a mask with only the area of the aruco tag
        cv2.fillPoly(self.arucos_mask, pts = rectangle_corners_for_mask, color=(255,255,255))  
        one_channel_arucos_mask = cv2.cvtColor(self.arucos_mask, cv2.COLOR_BGR2GRAY)  /255.0        
        self.arucos_mask_with_distance = np.nan_to_num(self.point_cloud_ocv)*one_channel_arucos_mask
        # step two - we get the mean point cloud value on the aruco tag area, to use it as z value
        tag_area = one_channel_arucos_mask.sum()
        if tag_area > 0:        
            z_center = (self.arucos_mask_with_distance/255.0).sum()/tag_area
        else:
            z_center = 0.0        
        # on the next line the minus signs allow us to transform the camera reference frame to a reference frame that
        # is parallel to the robots reference frame 
        return (float(z_center), float(-x_center), float(-y_center)) 

    def transform_aruco_midpoint_to_metric_system(self, aruco_midpoint): 
   
        x_m = (0.25738586)*(aruco_midpoint[0]) + 0.05862189

        x_px_times_y_px = aruco_midpoint[0]*aruco_midpoint[1]
        y_m = 0.29283879*aruco_midpoint[0] + 0.00050015*aruco_midpoint[1] + 0.00094536*x_px_times_y_px + 0.23096646

        x_px_times_z_px = aruco_midpoint[0]*aruco_midpoint[2]
        z_m = 0.16725805*aruco_midpoint[0] - 0.00069012*aruco_midpoint[2] + 0.00098029*x_px_times_z_px - 0.04520938         
        return (x_m, y_m, z_m)

    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        #print("cv2_to_imgmsg image shape is:" + str(image.shape))
        if encoding == "bgr8":
            self.curr_signs_image_msg.header = Header()
            self.curr_signs_image_msg.height = image.shape[0]
            self.curr_signs_image_msg.width = image.shape[1]
            self.curr_signs_image_msg.encoding = encoding
            self.curr_signs_image_msg.is_bigendian = 0
            self.curr_signs_image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.curr_signs_image_msg.height, self.curr_signs_image_msg.step) )
            data = np.reshape(image, (self.curr_signs_image_msg.height*self.curr_signs_image_msg.step) )
            data = list(data)
            self.curr_signs_image_msg.data = data
            return self.curr_signs_image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 
            
    def euclidean_distance(self, tuple):
        return math.sqrt(tuple[0]**2 + tuple[1]**2 + tuple[2]**2)

    def get_closest_point(self, point_list):
        # TODO - implement this functions logic
        euclidean_distances = list(map(self.euclidean_distance, point_list))
        positions_and_euclidean_distances = list(zip(point_list, euclidean_distances))
        positions_and_euclidean_distances.sort(key = lambda x:x[1])

        return positions_and_euclidean_distances[0][0]

        


    def tuple_position_2_ros_position(self, tuple_point):
        ros_point = Point()
        ros_point.x = tuple_point[0]
        ros_point.y = tuple_point[1]
        ros_point.z = tuple_point[2]
        return ros_point

    def filters (self, arucos_list):
        new_arcuso_list = []
        
        # verify *at least* one ArUco marker was detected
        if len(arucos_list) > 0:
			# loop over the detected ArUCo corners
            for aruco in arucos_list:
                corners = aruco.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                ############# square filter ##############
                #if self.square_filter(topRight, bottomRight, bottomLeft, topLeft):
                if self.twenty_cm_len_filter(corners):
                    new_arcuso_list.append(aruco)

            return tuple(new_arcuso_list)
        return arucos_list

    def transform_aruco_corners_to_metric_system(self, corners):                
        corners_in_metric_system = corners.copy()        
        aruco_depth_px, _, _ = self.get_aruco_midpoint(corners)
        #aruco_depth, _, _ = self.transform_aruco_midpoint_to_metric_system(aruco_midpoint)
        corners_in_metric_system = np.concatenate((corners_in_metric_system, np.ones((corners_in_metric_system.shape[0],1))), axis = 1)

        for i in range(corners.shape[0]):
            row = corners[i,:]
            # aruco_midpoint[0] -> aruco_depth_px
            # aruco_midpoint[1] -> -row[0]
            # aruco_midpoint[2] -> -row[1]
            x_m = (0.25738586)*(aruco_depth_px) + 0.05862189

            x_px_times_y_px = aruco_depth_px*(-row[0])
            y_m = 0.29283879*aruco_depth_px + 0.00050015*(-row[0]) + 0.00094536*x_px_times_y_px + 0.23096646

            x_px_times_z_px = aruco_depth_px*(-row[1])
            z_m = 0.16725805*aruco_depth_px - 0.00069012*(-row[1]) + 0.00098029*x_px_times_z_px - 0.04520938
            corners_in_metric_system[i] = [x_m, y_m, z_m]
        return corners_in_metric_system        

    def twenty_cm_len_filter(self, corners):
        corners_in_metric_system = self.transform_aruco_corners_to_metric_system(corners)
        aruco_side_lenghts = []
        for i in range(4):
            xyz_dist = corners_in_metric_system[ (i+1)%4 ] - corners_in_metric_system[i]
            xyz_dist = tuple(xyz_dist)
            aruco_side_lenghts.append( self.euclidean_distance(xyz_dist) )
        return abs(max(aruco_side_lenghts) - 0.2) <= 1.9 and abs(min(aruco_side_lenghts) - 0.2) <= 1.9

    def square_filter(self, tr, br, bl, tl):
        top = abs(tr[0] - tl[0])
        bottom = abs(br[0] - bl[0])
        right = abs(tr[1] - br[1])
        left = abs(tl[1] - bl[1])
        min_side = min(top,bottom,right,left)
        max_side = max(top,bottom,right,left)
        #print(max_side - min_side)
        is_square = max_side - min_side < self.square_filter_trh

        return is_square

    def main(self):
        while not rospy.is_shutdown():
            self.arucos_mask = np.zeros((self.image_size.height, self.image_size.width, 3), dtype = np.int8)
            self.arucos_mask_with_distance = np.zeros((self.image_size.height, self.image_size.width), dtype = np.float64)
            if self.zed_camera.grab(self.zed_runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                self.zed_camera.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                # Retrieve depth map. Depth is aligned on the left image
                self.zed_camera.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, self.image_size)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                self.zed_camera.retrieve_measure(self.point_cloud, sl.MEASURE.DEPTH, sl.MEM.CPU, self.image_size)
                self.image_ocv = self.image_zed.get_data()
                self.image_ocv = self.image_ocv[:,:,:-1]
                self.depth_image_ocv = self.depth_image_zed.get_data()
                self.point_cloud_ocv = self.point_cloud.get_data()

                aruco_corners, aruco_ids = self.get_arucos_info_in_image(self.image_ocv)
                aruco_corners = self.filters(aruco_corners)
                
                self.displayed_image_ocv = self.image_ocv.copy()
                if len(aruco_corners) > 0:
                    self.displayed_image_ocv = self.draw_arucos(self.displayed_image_ocv, aruco_corners)                
                    aruco_centers = list(map(self.get_aruco_midpoint, aruco_corners))

                    centers_meters = list(map(self.transform_aruco_midpoint_to_metric_system, aruco_centers))           
                    closest_aruco_position = self.get_closest_point(centers_meters)

                    #closest_aruco_position = self.transform_aruco_midpoint_to_metric_system(closest_aruco_position)                
                    self.closest_aruco_position_publisher.publish( self.tuple_position_2_ros_position(closest_aruco_position))

                self.displayed_image_ocv = cv2.resize(self.displayed_image_ocv, (130,100), interpolation = cv2.INTER_AREA) 
                self.curr_signs_image_msg = self.cv2_to_imgmsg(self.displayed_image_ocv, encoding = "bgr8")
                self.image_pub.publish(self.curr_signs_image_msg)
                self.curr_signs_image_msg_2 = self.cv2_to_imgmsg(self.arucos_mask, encoding = "bgr8")
                self.image_aruco_mask.publish(self.curr_signs_image_msg_2)                
                three_channel_point_cloud = cv2.merge(
                    (np.floor(np.nan_to_num(self.point_cloud_ocv)).astype(np.uint8), 
                     np.floor(np.nan_to_num(self.point_cloud_ocv)).astype(np.uint8),
                     np.floor(np.nan_to_num(self.point_cloud_ocv)).astype(np.uint8))
                    )
                #print("depth image shape is: {d}".format(d = self.depth_image_zed_ocv.shape))
                self.curr_signs_image_msg_4 = self.cv2_to_imgmsg(three_channel_point_cloud, encoding = "bgr8")
                self.image_point_cloud_ocv.publish(self.curr_signs_image_msg_4)
                """ self.curr_signs_image_msg_3 = self.cv2_to_imgmsg(self.arucos_mask_with_distance, encoding = "bgr8")
                self.image_aruco_mask_distance.publish(self.curr_signs_image_msg_3) """ 

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    aruco_detector.main()
