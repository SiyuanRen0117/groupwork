#include <cw1_team_13/task2.h>

////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init () before using any other
    * part of the ROS system.
    */
  ros::init (argc, argv, "cw1_team_13_task2_node");
  
  /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");
  
  // Create a PCL object
  PCLTask pcl_Object (nh);
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_cloud =
    nh.subscribe ("/r200/camera/depth_registered/points",
                  1,
                  &PCLTask::cloudCallBackOne,
                  &pcl_Object);
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Loop at 30Hz
  ros::Rate loop_rate (30);
  
  while (ros::ok ())
  {
    // ROS Spin
    ros::spinOnce ();
    
    // Sleep
    loop_rate.sleep ();
  }
  
  return (0);
}
