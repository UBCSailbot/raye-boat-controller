#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_boat_controller_sailbot_msg_Sensors and sailbot_msg::Sensors

void convertFromBus(sailbot_msg::Sensors* msgPtr, SL_Bus_boat_controller_sailbot_msg_Sensors const* busPtr)
{
  const std::string rosMessageType("sailbot_msg/Sensors");

  msgPtr->accelerometer_x_axis_acceleration =  busPtr->AccelerometerXAxisAcceleration;
  msgPtr->accelerometer_y_axis_acceleration =  busPtr->AccelerometerYAxisAcceleration;
  msgPtr->accelerometer_z_axis_acceleration =  busPtr->AccelerometerZAxisAcceleration;
  msgPtr->boom_angle_sensor_angle =  busPtr->BoomAngleSensorAngle;
  msgPtr->gps_0_groundspeed =  busPtr->Gps0Groundspeed;
  msgPtr->gps_0_latitude =  busPtr->Gps0Latitude;
  msgPtr->gps_0_latitude_loc =  busPtr->Gps0LatitudeLoc;
  msgPtr->gps_0_longitude =  busPtr->Gps0Longitude;
  msgPtr->gps_0_longitude_loc =  busPtr->Gps0LongitudeLoc;
  msgPtr->gps_0_magnetic_variation =  busPtr->Gps0MagneticVariation;
  msgPtr->gps_0_magnetic_variation_sense =  busPtr->Gps0MagneticVariationSense;
  convertFromBusVariablePrimitiveArray(msgPtr->gps_0_timestamp, busPtr->Gps0Timestamp, busPtr->Gps0Timestamp_SL_Info);
  msgPtr->gps_0_track_made_good =  busPtr->Gps0TrackMadeGood;
  msgPtr->gps_0_true_heading =  busPtr->Gps0TrueHeading;
  msgPtr->gps_1_groundspeed =  busPtr->Gps1Groundspeed;
  msgPtr->gps_1_latitude =  busPtr->Gps1Latitude;
  msgPtr->gps_1_latitude_loc =  busPtr->Gps1LatitudeLoc;
  msgPtr->gps_1_longitude =  busPtr->Gps1Longitude;
  msgPtr->gps_1_longitude_loc =  busPtr->Gps1LongitudeLoc;
  msgPtr->gps_1_magnetic_variation =  busPtr->Gps1MagneticVariation;
  msgPtr->gps_1_magnetic_variation_sense =  busPtr->Gps1MagneticVariationSense;
  convertFromBusVariablePrimitiveArray(msgPtr->gps_1_timestamp, busPtr->Gps1Timestamp, busPtr->Gps1Timestamp_SL_Info);
  msgPtr->gps_1_track_made_good =  busPtr->Gps1TrackMadeGood;
  msgPtr->gps_1_true_heading =  busPtr->Gps1TrueHeading;
  msgPtr->wind_sensor_0_direction =  busPtr->WindSensor0Direction;
  msgPtr->wind_sensor_0_reference =  busPtr->WindSensor0Reference;
  msgPtr->wind_sensor_0_speed =  busPtr->WindSensor0Speed;
  msgPtr->wind_sensor_1_direction =  busPtr->WindSensor1Direction;
  msgPtr->wind_sensor_1_reference =  busPtr->WindSensor1Reference;
  msgPtr->wind_sensor_1_speed =  busPtr->WindSensor1Speed;
  msgPtr->wind_sensor_2_direction =  busPtr->WindSensor2Direction;
  msgPtr->wind_sensor_2_reference =  busPtr->WindSensor2Reference;
  msgPtr->wind_sensor_2_speed =  busPtr->WindSensor2Speed;
}

void convertToBus(SL_Bus_boat_controller_sailbot_msg_Sensors* busPtr, sailbot_msg::Sensors const* msgPtr)
{
  const std::string rosMessageType("sailbot_msg/Sensors");

  busPtr->AccelerometerXAxisAcceleration =  msgPtr->accelerometer_x_axis_acceleration;
  busPtr->AccelerometerYAxisAcceleration =  msgPtr->accelerometer_y_axis_acceleration;
  busPtr->AccelerometerZAxisAcceleration =  msgPtr->accelerometer_z_axis_acceleration;
  busPtr->BoomAngleSensorAngle =  msgPtr->boom_angle_sensor_angle;
  busPtr->Gps0Groundspeed =  msgPtr->gps_0_groundspeed;
  busPtr->Gps0Latitude =  msgPtr->gps_0_latitude;
  busPtr->Gps0LatitudeLoc =  msgPtr->gps_0_latitude_loc;
  busPtr->Gps0Longitude =  msgPtr->gps_0_longitude;
  busPtr->Gps0LongitudeLoc =  msgPtr->gps_0_longitude_loc;
  busPtr->Gps0MagneticVariation =  msgPtr->gps_0_magnetic_variation;
  busPtr->Gps0MagneticVariationSense =  msgPtr->gps_0_magnetic_variation_sense;
  convertToBusVariablePrimitiveArray(busPtr->Gps0Timestamp, busPtr->Gps0Timestamp_SL_Info, msgPtr->gps_0_timestamp, slros::EnabledWarning(rosMessageType, "gps_0_timestamp"));
  busPtr->Gps0TrackMadeGood =  msgPtr->gps_0_track_made_good;
  busPtr->Gps0TrueHeading =  msgPtr->gps_0_true_heading;
  busPtr->Gps1Groundspeed =  msgPtr->gps_1_groundspeed;
  busPtr->Gps1Latitude =  msgPtr->gps_1_latitude;
  busPtr->Gps1LatitudeLoc =  msgPtr->gps_1_latitude_loc;
  busPtr->Gps1Longitude =  msgPtr->gps_1_longitude;
  busPtr->Gps1LongitudeLoc =  msgPtr->gps_1_longitude_loc;
  busPtr->Gps1MagneticVariation =  msgPtr->gps_1_magnetic_variation;
  busPtr->Gps1MagneticVariationSense =  msgPtr->gps_1_magnetic_variation_sense;
  convertToBusVariablePrimitiveArray(busPtr->Gps1Timestamp, busPtr->Gps1Timestamp_SL_Info, msgPtr->gps_1_timestamp, slros::EnabledWarning(rosMessageType, "gps_1_timestamp"));
  busPtr->Gps1TrackMadeGood =  msgPtr->gps_1_track_made_good;
  busPtr->Gps1TrueHeading =  msgPtr->gps_1_true_heading;
  busPtr->WindSensor0Direction =  msgPtr->wind_sensor_0_direction;
  busPtr->WindSensor0Reference =  msgPtr->wind_sensor_0_reference;
  busPtr->WindSensor0Speed =  msgPtr->wind_sensor_0_speed;
  busPtr->WindSensor1Direction =  msgPtr->wind_sensor_1_direction;
  busPtr->WindSensor1Reference =  msgPtr->wind_sensor_1_reference;
  busPtr->WindSensor1Speed =  msgPtr->wind_sensor_1_speed;
  busPtr->WindSensor2Direction =  msgPtr->wind_sensor_2_direction;
  busPtr->WindSensor2Reference =  msgPtr->wind_sensor_2_reference;
  busPtr->WindSensor2Speed =  msgPtr->wind_sensor_2_speed;
}


// Conversions between SL_Bus_boat_controller_sailbot_msg_actuation_angle and sailbot_msg::actuation_angle

void convertFromBus(sailbot_msg::actuation_angle* msgPtr, SL_Bus_boat_controller_sailbot_msg_actuation_angle const* busPtr)
{
  const std::string rosMessageType("sailbot_msg/actuation_angle");

  msgPtr->rudder =  busPtr->Rudder;
  msgPtr->winch =  busPtr->Winch;
}

void convertToBus(SL_Bus_boat_controller_sailbot_msg_actuation_angle* busPtr, sailbot_msg::actuation_angle const* msgPtr)
{
  const std::string rosMessageType("sailbot_msg/actuation_angle");

  busPtr->Rudder =  msgPtr->rudder;
  busPtr->Winch =  msgPtr->winch;
}


// Conversions between SL_Bus_boat_controller_sailbot_msg_heading and sailbot_msg::heading

void convertFromBus(sailbot_msg::heading* msgPtr, SL_Bus_boat_controller_sailbot_msg_heading const* busPtr)
{
  const std::string rosMessageType("sailbot_msg/heading");

  msgPtr->headingDegrees =  busPtr->HeadingDegrees;
}

void convertToBus(SL_Bus_boat_controller_sailbot_msg_heading* busPtr, sailbot_msg::heading const* msgPtr)
{
  const std::string rosMessageType("sailbot_msg/heading");

  busPtr->HeadingDegrees =  msgPtr->headingDegrees;
}

