# Project: 3D Perception - pr2 robot 
Before starting perception pipeline description, we build the project as explained in RoboND-Perception-Project readme file 
---
**Then our work main steps are:**
1. Filtering and RANSAC plane fitting. 
2. Clustering for segmentation.
3. Features extraction.  
4. SVM trainning.
5. Object Recognition. 
6. Rotating Robot for Collision Map creation 
7. Collision Map creation 
8. Output yaml files  
9. Pick and Place 

[//]: # (Image References) 
[image1]: ./images/Figure_1.png
[image2]: ./images/Figure_2.png
[image3]: ./images/Figure_3.png
[image4]: ./images/Figure_4.png
[image5]: ./images/Figure_5.png
[image6]: ./images/Figure_6.png
[image7]: ./images/Figure_7.png
[image8]: ./images/Figure_8.png
[image9]: ./images/Figure_9.png
[image10]: ./images/Figure_10.png
[image11]: ./images/Figure_11.png


## We explain each of these steps below 
### 1. Filtering and RANSAC plane fitting
Subscriber gets pointcloud of the scene captured by pr2_robot camera. pcl_callback is executed as an action.
Four steps are performed before clustering. 1st step is voxel downsampling of the point cloud.
2nd step is outlier removal by statistical filter. 3rd step use pass through filter to determing certain range of the scene 
to deal with. Lastly, 4th step is to use RANSAC to split the plane (representing the table) and objects above it. 
```
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
```
![alt_text][image1]

![alt_text][image2]

### 2. Clustering for segmentation
Use clustering algorithm (Euclidean Cluster) to cluster the different objects of interest 
so that each one is classified to one of objects listed in the pick list 
```
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
```
![alt_text][image3]

### 3. Features extraction
First we launch the training.launch file to bring up the Gazebo environment
`roslaunch sensor_stick training.launch`
Then we capture features using the function given in sensor_stick
`rosrun sensor_stick capture_features.py`
This will generate features for us and save them in training_set.sav file 
The features are color histograms (with nbin = 32) and 32 normal histograms (with nbin = 32)
We compute these features by the two functions `compute_color_histograms` and `compute_normal_histograms` in features.py 
```
def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    r_hist = np.histogram(channel_1_vals, bins=32, range=(0, 256))
    g_hist = np.histogram(channel_2_vals, bins=32, range=(0, 256))
    b_hist = np.histogram(channel_3_vals, bins=32, range=(0, 256))
    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((r_hist[0], g_hist[0], b_hist[0])).astype(np.float64)
    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    normed_features = hist_features / np.sum(hist_features)
    return normed_features 
```
```
def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    x_hist = np.histogram(norm_x_vals, bins=32, range=(0, 256))
    y_hist = np.histogram(norm_y_vals, bins=32, range=(0, 256))
    z_hist = np.histogram(norm_z_vals, bins=32, range=(0, 256))
    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((x_hist[0], y_hist[0], z_hist[0])).astype(np.float64)
    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    normed_features = hist_features / np.sum(hist_features)

    return normed_features
```
### 4. SVM Training
We use the function train_svm.py to train our model and save it in model.sav file 
`rosrun sensor_stick train_svm.py`
The confusion matrix for our model is shown 

![alt_text][image4] ![alt_text][image5]

### 5. Object Recognition
A trained SVM is used to classify different objects (different clusters from segmentation step) 

```
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
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
```
![alt_text][image6]

### 6. Rotating Robot for Collision Map creation
We rotate the robot by publishing joint angle through the topic `/pr2/world_joint_controller/command` 
to scan the environment so that it can build the collision map

![alt_text][image7]

### 7. Collision Map creation 
collision map is constructed by publishing the point cloud of the table and all objects (that not picked yet)
except the one to be picked. Publishing to the topic: `/pr2/3d_map/points` allow the mapping algorithm to consider this pointcloud
as obstacles. **Note:** to concatenate pointclouds for collision map creation, I used a function written by another one `ros_to_pcl2` 
which is written in the `pcl_helper.py` file. So I uploaded a version of the modified file.  

![alt_text][image8]

### 8. Output yaml files
Three .yaml files: `output_1.yaml`, `output_2.yaml`, `output_3.yaml` for the given three worlds are produced. 
Object Recognition for the three worlds is shown below 

![alt_text][image9]
![alt_text][image10]
![alt_text][image11]

### 9. Pick and Place 
I tried everything. All the phases are running well and the collision map is constructed as specified,
but unfortunately everytime the call of `pick_place_routine` rosservice return resp.success with False. 
I have no idea about the cause for this 
