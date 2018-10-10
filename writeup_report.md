# Project: 3D Perception - pr2 robot 
Before starting perception pipeline description, we build the project as explained in RoboND-Perception-Project readme file 
---
**Then our work main steps are:**
1. Filtering and RANSAC plane fitting. 
2. Clustering for segmentation.
3. Features extraction and SVM trainning. 
4. Object Recognition. 
5. Rotating Robot for Collision Map creation 
6. Collision Map creation 
7. Output yaml files  
8. Pick and Place 

[//]: # (Image References) 
[image1]: ./images/Figure_1.png
[image2]: ./images/Figure_2.png
[image3]: ./images/Figure_3.png
[image4]: ./images/Figure_4.png
[image5]: ./images/Figure_5.png
[image6]: ./images/Figure_6.png
[image7]: ./images/Figure_7.png
[image8]: ./images/Figure_8.png



## We explain each of these steps below 
### 1. Filtering and RANSAC plane fitting
Subscriber gets pointcloud of the scene captured by pr2_robot camera. pcl_callback is executed as an action.
Four steps are performed before clustering. 1st step is voxel downsampling of the point cloud.
2nd step is outlier removal by statistical filter. 3rd step use pass through filter to determing certain range of the scene 
to deal with. Lastly, 4th step is to use RANSAC to split the plane (representing the table) and objects above it. 

![alt_text][image1]

![alt_text][image2]

### 2. Clustering for segmentation
Use clustering algorithm (Euclidean Cluster) to cluster the different objects of interest 
so that each one is classified to one of objects listed in the pick list 

![alt_text][image3]

### 3. Features extraction and SVM training 
First we launch the training.launch file to bring up the Gazebo environment
`roslaunch sensor_stick training.launch`
Then we capture features using the function given in sensor_stick
`rosrun sensor_stick capture_features.py`
This will generate features for us and save them in training_set.sav file 
finally we use the function train_svm.py to train our model and save it in model.sav file 
`osrun sensor_stick train_svm.py`
The confusion matrix for our model is shown 

![alt_text][image4] ![alt_text][image5]
### 4. Object Recognition
A trained SVM is used to classify different objects (different clusters from segmentation step) 

![alt_text][image6]

### 5. Rotating Robot for Collision Map creation
We rotate the robot by publishing joint angle through the topic `/pr2/world_joint_controller/command` 
to scan the environment so that it can build the collision map

![alt_text][image7]

### 6. Collision Map creation 
collision map is constructed by publishing the point cloud of the table and all objects (that not picked yet)
except the one to be picked. Publishing to the topic: `/pr2/3d_map/points` allow the mapping algorithm to consider this pointcloud
as obstacles. **Note:** to concatenate pointclouds for collision map creation, I used a function written by another one `ros_to_pcl2` 
which is written in the `pcl_helper.py` file. So I uploaded a version of the modified file.  

![alt_text][image8]

### 7. Output yaml files
Three .yaml files: `output_1.yaml`, `output_2.yaml`, `output_3.yaml` for the given three worlds are produced. 

### 8. Pick and Place 
I tried everything. All the phases are running well and the collision map is constructed as specified,
but unfortunately everytime the call of `pick_place_routine` rosservice retrun resp.success with False. 
I have no idea about the cause for this 
