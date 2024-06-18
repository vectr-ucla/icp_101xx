/*
  Author: Brett Lopez
  Contact: btlopez@ucla.edu
  Date: Oct 3, 2022
*/

#include "RosManager.hpp"
#include "signal.h"

// Prototypes
void controlC(int sig);

int main(int argc, char **argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "icp_101xx_ros_node");
  ros::NodeHandle nh("~");

  /* Setup the CTRL-C trap */
  signal(SIGINT, controlC);
  sleep(0.5);

  // Initialize class here
  RosManager::RosNode node(nh);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  node.Start();
  ros::waitForShutdown();

  return 0;
}

//## Custom Control-C handler
void controlC(int sig) {
  // Kill node here
  RosManager::RosNode::Abort();
}
