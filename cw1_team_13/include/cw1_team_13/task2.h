#ifndef PCL_TUTORIAL_H_
#define PCL_TUTORIAL_H_


#pragma once


#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
#include <algorithm> 
#include <iostream>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// headers generated by catkin for the custom services we have made
#include <cw1_team_13/task2_start.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

/** \brief PCL Task
  *
  * \author Dimitrios Kanoulas
  */
class PCLTask
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    PCLTask (ros::NodeHandle &nh);
    
    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackOne (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);


    bool
    task2_startCallback(cw1_team_13::task2_start::Request &request,
      cw1_team_13::task2_start::Response &response);
    void
    movecamera_left_to_right();

    bool
    moveArm(geometry_msgs::Pose target_pose);

    void
    checkcube(PointCPtr &in_cloud_ptr);

    void
    addCollisionObject(std::string object_name,
      geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
      geometry_msgs::Quaternion orientation);

    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyVX (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);

    /** \brief Apply Pass Through filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyPT (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);
    
    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals (PointCPtr &in_cloud_ptr);
    
    /** \brief Segment Plane from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane (PointCPtr &in_cloud_ptr);
    
    
    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findCylPose (PointCPtr &in_cloud_ptr);
    
    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
    
    /** \brief Publish the cylinder point.
      * 
      *  \input[in] cyl_pt_msg Cylinder's geometry point
      *  
      *  \output true if three numbers are added
      */
    void
    publishPose (geometry_msgs::PointStamped &cyl_pt_msg);
    
  public:
    /** \brief Node handle. */
    ros::NodeHandle g_nh;

    /** \brief  service servers for advertising ROS services  */
    ros::ServiceServer task2_start_srv_;

    /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

    /** \brief MoveIt interface to interact with the moveit planning scene 
      * (eg collision objects). */
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    
    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud;
    
    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_cyl_pt_msg;
    
    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_pose;
    
    /** \brief Voxel Grid filter's leaf size. */
    double g_vg_leaf_sz;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered2;
    
    /** \brief Point Cloud (filtered) sensros_msg for publ. */
    sensor_msgs::PointCloud2 g_cloud_filtered_msg;
    
    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;
    
    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> g_vx;
    
    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> g_pt;
    
    /** \brief Pass Through min and max threshold sizes. */
    double g_pt_thrs_min, g_pt_thrs_max;
    
    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    
    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    
    /** \brief Nearest neighborhooh size for normal estimation. */
    double g_k_nn;
    
    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
    
    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> g_extract_pc;
  
    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
    /** \brief Point indices for plane. */
    pcl::PointIndices::Ptr g_inliers_plane;
      
    /** \brief Point indices for cylinder. */
    pcl::PointIndices::Ptr g_inliers_cylinder;
    
    /** \brief Model coefficients for the plane segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    
    /** \brief Model coefficients for the culinder segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_cylinder;
    
    /** \brief Point cloud to hold plane and cylinder points. */
    PointCPtr g_cloud_plane, g_cloud_cylinder;

    /** \brief Point cloud to store cloud points. */
    PointCPtr g_cloud_store;

    
    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_;
    
    /* Variables */

    /** \brief Define some useful constant values */
    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;

    /** \brief Parameters to define the pick operation */
    double z_offset_ = 0.125;
    double angle_offset_ = 3.14159 / 4.0;
    double approach_distance_ = 0.1;


  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif