#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block boat_controller/Subscribe
extern SimulinkSubscriber<sailbot_msg::Sensors, SL_Bus_boat_controller_sailbot_msg_Sensors> Sub_boat_controller_97;

// For Block boat_controller/Subscribe1
extern SimulinkSubscriber<sailbot_msg::heading, SL_Bus_boat_controller_sailbot_msg_heading> Sub_boat_controller_192;

// For Block boat_controller/Publish
extern SimulinkPublisher<sailbot_msg::actuation_angle, SL_Bus_boat_controller_sailbot_msg_actuation_angle> Pub_boat_controller_103;

void slros_node_init(int argc, char** argv);

#endif
