#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
from sensor_msgs.msg import JointState
import yaml
from sensor_stick.pcl_helper import ros_to_pcl2

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    
    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    
    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    pcl_filtered = vox.filter()
    
    # TODO: Statistical Outlier Filter 
    outlier_filter = pcl_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.05
    outlier_filter.set_std_dev_mul_thresh(x)
    pcl_filtered = outlier_filter.filter()
    
    # TODO: PassThrough Filter
    passthrough = pcl_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    pcl_filtered = passthrough.filter()
    
    passthrough = pcl_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 2
    passthrough.set_filter_limits(axis_min, axis_max)
    pcl_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = pcl_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers
    cloud_table = pcl_filtered.extract(inliers, negative=False)
    cloud_objects = pcl_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.025)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
    	for i, indice in enumerate(indices):
    	    color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    global collision_map 
    collision_map = cloud_table
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
	ros_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
	chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .3
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    
    try:
        if len(detected_objects) > 0:
            pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    
   
    return 
# function to load parameters and request PickPlace service
def pr2_mover(objects_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    test_scene_num.data = 4
    object_name = String()
    object_group = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    global collision_map
    	
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    #get the coordinates of left and right dropbox 
    dropbox_param = rospy.get_param('/dropbox')
    left_place = dropbox_param[0]['position']
    right_place = dropbox_param[1]['position']
   
    # TODO: Rotate PR2 in place to capture side tables for the collision map
    global reached_right
    global reached_left
    global reached_center 
    global loc
    if reached_center == False:
        if reached_right == False:
            rotate_robot.publish(-1.75)
            joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
            loc = joint_state.position[19]
            if abs(-1.75 - loc) <= 0.05: 
                reached_right = True 
        elif reached_left == False:
    	    rotate_robot.publish(1.75)
    	    joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
            loc = joint_state.position[19]
            if abs(1.75 - loc) <= 0.05: 
                reached_left = True 
    	else:
    	    rotate_robot.publish(0.0)
    	    joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
            loc = joint_state.position[19]
            if abs(0.0 - loc) <= 0.05: 
                reached_center = True
     
    # TODO: Loop through the pick list
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    dict_list = []
    for i in range(len(object_list_param)):
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        object_name.data = object_list_param[i]['name']
        object_group.data = object_list_param[i]['group']
        
        # add individual pointclouds to the collidable map to be used for path planning
        # the current object to be picked is not in the collidable map 
        ros_collidable_map = pcl_to_ros(collision_map)
        for obj in objects_list:
            if object_name.data != obj.label:
                pcl_collidable_map = ros_to_pcl2(ros_collidable_map,obj.cloud)
                ros_collidable_map = pcl_to_ros(pcl_collidable_map)
        sensed_objects.publish(ros_collidable_map)
        
        # loop through detected objects to match with one in pick list and get its centroid 
        for obj in objects_list:
            if object_name.data == obj.label:
    	        points_arr = ros_to_pcl(obj.cloud).to_array()
    	        centroid = np.mean(points_arr, axis=0)[:3]
   		centroid = [np.asscalar(centroid[0]),np.asscalar(centroid[1]),np.asscalar(centroid[2])] 	        
                pick_pose.position.x = centroid[0]
                pick_pose.position.y = centroid[1]
                pick_pose.position.z = centroid[2]           
		# TODO: Create 'place_pose' for the object
		# TODO: Assign the arm to be used for pick_place
		if object_group.data == 'red':
		    arm_name.data = 'left'
		    place_pose.position.x = left_place[0]
		    place_pose.position.y = left_place[1]
		    place_pose.position.z = left_place[2]
		else:
		    arm_name.data = 'right'
		    place_pose.position.x = right_place[0]
		    place_pose.position.y = right_place[1]
		    place_pose.position.z = right_place[2]
		# TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
		yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
		dict_list.append(yaml_dict)
   
        # Wait for 'pick_place_routine' service to come up
        if reached_center == True:
            rospy.wait_for_service('pick_place_routine')
            try:        
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)

                print(resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s" %e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_%d.yaml' % test_scene_num.data, dict_list)
    print("Done")
    return 

if __name__ == '__main__':

    #gloabal vaiables 
    reached_right = False 
    reached_left = False 
    reached_center = False 
    loc = 0.0
    collision_map = pcl.PointCloud()
    
    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=12)
    
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=12)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=12)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=12)
    object_markers_pub = rospy.Publisher("/object_markers",Marker,queue_size=12)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=12)
    sensed_objects = rospy.Publisher("/pr2/3d_map/points",PointCloud2,queue_size=12)
    rotate_robot = rospy.Publisher("/pr2/world_joint_controller/command",Float64,queue_size=12)
    
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
    	rospy.spin()
