#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "boat_controller";

// For Block boat_controller/Subscribe
SimulinkSubscriber<sailbot_msg::Sensors, SL_Bus_boat_controller_sailbot_msg_Sensors> Sub_boat_controller_97;

// For Block boat_controller/Subscribe1
SimulinkSubscriber<sailbot_msg::heading, SL_Bus_boat_controller_sailbot_msg_heading> Sub_boat_controller_192;

// For Block boat_controller/Publish
SimulinkPublisher<sailbot_msg::actuation_angle, SL_Bus_boat_controller_sailbot_msg_actuation_angle> Pub_boat_controller_103;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

