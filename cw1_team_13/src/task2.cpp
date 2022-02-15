#include <cw1_team_13/task2.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

////////////////////////////////////////////////////////////////////////////////
PCLTask::PCLTask(ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  //GCLOUD STORE
  g_cloud_store (new PointC),
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  debug_ (false)
{
  g_nh = nh;
  //advertise service
  task2_start_srv_ = g_nh.advertiseService("/task2_start",
    &PCLTask::task2_startCallback, this); 
  
  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  //********************CHANGE POSE HERE*****************************8
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.45; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);
  applyPT (g_cloud_ptr, g_cloud_filtered);
  
  // Segment plane
  findNormals (g_cloud_filtered);
  segPlane (g_cloud_filtered);
  
  //findCylPose (g_cloud_cylinder);
    
  // Publish the data
  //ROS_INFO ("Publishing Filtered Cloud 2");
  //pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_plane);
  
  return;
}
////////////////////////////////////////////////////////////////////////////////
bool
PCLTask::task2_startCallback(cw1_team_13::task2_start::Request &request,
  cw1_team_13::task2_start::Response &response)
{
  /* This service picks up the robot */
  ROS_INFO("Service Advertised");
  ROS_INFO_STREAM(request);
  //MOVE CAMERA//
  std::string object_ground = "ground";
  geometry_msgs::Point ground_centre;
  ground_centre.x = 0.1;
  ground_centre.y = 0.0;
  ground_centre.z = 0.005;
  geometry_msgs::Vector3 ground_dimension;
  ground_dimension.x = 1.1;
  ground_dimension.y = 0.8;
  ground_dimension.z = 0.01;
  geometry_msgs::Quaternion ground_orientation;
  ground_orientation.x = 0;
  ground_orientation.y = 0;
  ground_orientation.z = 0;
  ground_orientation.w = 0;

  addCollisionObject(object_ground,ground_centre,ground_dimension,
    ground_orientation);

  movecamera_left_to_right();
 
  
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::movecamera_left_to_right(){
  tf2::Quaternion q_x180deg(-1,0,0,0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);
  
  /*
  geometry_msgs::Pose corner_right_up;
  corner_right_up.position.x = -0.4;
  corner_right_up.position.y = 0.4;
  corner_right_up.position.z = 0.4;
  corner_right_up.orientation = grasp_orientation;
  */
  geometry_msgs::Pose corner_left_down;
  corner_left_down.position.x = 0.45;
  corner_left_down.position.y = -0.4;
  corner_left_down.position.z = 0.56;
  corner_left_down.orientation = grasp_orientation;
  

  //left close to right 
    for (int i = 0; i < 12; i++) {
      bool success_move = moveArm(corner_left_down);
      corner_left_down.position.y += 0.05;
      ROS_INFO_STREAM(i);
      ROS_INFO_STREAM(success_move);
      ROS_INFO("SUCCESSSSSSSSSSSSSSSSSSSSS MOVE corner_right_down");
      //bug
      if (i%5 == 0){
        // STORE GLOUD FOR CHECKCUBE !!!!!!!
        //checkcube(*g_cloud_filtered);
        ROS_INFO_STREAM(i);
        ROS_INFO("SUCCESSSSSSSSSSSSSSSSSSSSS check");
      }
      //checkcube(g_cloud_plane);
      ROS_INFO("SUCCESSSSSSSSSSSSSSSSSSSSS check");
    } 

}


///////////////////////////////////////////////////////////////////////////////
bool
PCLTask::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  // plane or normal plane?
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);

  int num = g_inliers_plane->indices.size();
  ROS_INFO_STREAM(num);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size ()
                   << " data points.");
}
    
////////////////////////////////////////////////////////////////////////////////
void
PCLTask::checkcube(PointCPtr &in_cloud_ptr)
{
  int size = in_cloud_ptr->size ();
  float points_r [size];
  float points_g [size];
  float points_b [size];
  std::vector<int> points_r_unique;
  //int points_r_unique [size];
  //CHANGE HERE***********
  int points_g_unique [size];
  int points_b_unique [size];

  
  for (int i = 0; i < size; i++) {
    int r = (int)in_cloud_ptr->points[i].r;
    int g = (int)in_cloud_ptr->points[i].g;
    int b = (int)in_cloud_ptr->points[i].b;
    points_r[i] = r/255;
    points_g[i] = g/255;
    points_b[i] = b/255;
    points_r_unique[i] = r;
    points_g_unique[i] = g;
    points_b_unique[i] = b;
  }
  std::sort(points_r_unique.begin(), points_r_unique.end());
  points_r_unique.erase(std::unique( points_r_unique.begin(), 
    points_r_unique.end() ), points_r_unique.end() );
  //int test [] = points_r_unique;
  std::vector<int> vector;
  vector = points_r_unique;

  int arr[vector.size()];
    
  for(int i = 0; i < vector.size(); i++) 
  {
      arr[i] = vector[i];
  }
  //ROS_INFO_STREAM(arr);


}

  



////////////////////////////////////////////////////////////////////////////////
void
PCLTask::findCylPose (PointCPtr &in_cloud_ptr)
{
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time (0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0",  // bad styling
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  publishPose (g_cyl_pt_msg_out);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
PCLTask::publishPose (geometry_msgs::PointStamped &cyl_pt_msg)
{
  // Create and publish the cylinder pose (ignore orientation)

  g_pub_pose.publish (cyl_pt_msg);
  
  return;
}

