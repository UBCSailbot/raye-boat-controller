#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <sailbot_msg/Sensors.h>
#include <sailbot_msg/actuation_angle.h>
#include <sailbot_msg/heading.h>
#include "boat_controller_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(sailbot_msg::Sensors* msgPtr, SL_Bus_boat_controller_sailbot_msg_Sensors const* busPtr);
void convertToBus(SL_Bus_boat_controller_sailbot_msg_Sensors* busPtr, sailbot_msg::Sensors const* msgPtr);

void convertFromBus(sailbot_msg::actuation_angle* msgPtr, SL_Bus_boat_controller_sailbot_msg_actuation_angle const* busPtr);
void convertToBus(SL_Bus_boat_controller_sailbot_msg_actuation_angle* busPtr, sailbot_msg::actuation_angle const* msgPtr);

void convertFromBus(sailbot_msg::heading* msgPtr, SL_Bus_boat_controller_sailbot_msg_heading const* busPtr);
void convertToBus(SL_Bus_boat_controller_sailbot_msg_heading* busPtr, sailbot_msg::heading const* msgPtr);


#endif
