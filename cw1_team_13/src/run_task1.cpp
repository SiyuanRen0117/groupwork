#include <cw1_team_13/task1.h>

int main(int argc, char** argv)
{
  // initialise ros and the node
  ros::init(argc, argv, "cw1_team_13_node");
  ros::NodeHandle nh("~");
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create the service object to handle all callbacks
  SrvClass myobject(nh);

  // loop rate in Hz
  ros::Rate rate(10);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();
    
    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

