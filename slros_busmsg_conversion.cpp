#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_boat_controller_sailbot_msg_Sensors and sailbot_msg::Sensors

void convertFromBus(sailbot_msg::Sensors* msgPtr, SL_Bus_boat_controller_sailbot_msg_Sensors const* busPtr)
{
  const std::string rosMessageType("sailbot_msg/Sensors");

  msgPtr->accelerometer_x_force_millig =  busPtr->AccelerometerXForceMillig;
  msgPtr->accelerometer_y_force_millig =  busPtr->AccelerometerYForceMillig;
  msgPtr->accelerometer_z_force_millig =  busPtr->AccelerometerZForceMillig;
  msgPtr->gps_ais_groundspeed_knots =  busPtr->GpsAisGroundspeedKnots;
  msgPtr->gps_ais_latitude_degreeMinutes =  busPtr->GpsAisLatitudeDegreeMinutes;
  msgPtr->gps_ais_longitude_degreeMinutes =  busPtr->GpsAisLongitudeDegreeMinutes;
  msgPtr->gps_ais_magnetic_variation_degrees =  busPtr->GpsAisMagneticVariationDegrees;
  msgPtr->gps_ais_state =  busPtr->GpsAisState;
  convertFromBusVariablePrimitiveArray(msgPtr->gps_ais_timestamp_utc, busPtr->GpsAisTimestampUtc, busPtr->GpsAisTimestampUtc_SL_Info);
  msgPtr->gps_ais_track_made_good_degrees =  busPtr->GpsAisTrackMadeGoodDegrees;
  msgPtr->gps_ais_true_heading_degrees =  busPtr->GpsAisTrueHeadingDegrees;
  msgPtr->gps_can_groundspeed_knots =  busPtr->GpsCanGroundspeedKnots;
  msgPtr->gps_can_latitude_degreeMinutes =  busPtr->GpsCanLatitudeDegreeMinutes;
  msgPtr->gps_can_longitude_degreeMinutes =  busPtr->GpsCanLongitudeDegreeMinutes;
  msgPtr->gps_can_magnetic_variation_degrees =  busPtr->GpsCanMagneticVariationDegrees;
  msgPtr->gps_can_state =  busPtr->GpsCanState;
  convertFromBusVariablePrimitiveArray(msgPtr->gps_can_timestamp_utc, busPtr->GpsCanTimestampUtc, busPtr->GpsCanTimestampUtc_SL_Info);
  msgPtr->gps_can_track_made_good_degrees =  busPtr->GpsCanTrackMadeGoodDegrees;
  msgPtr->gps_can_true_heading_degrees =  busPtr->GpsCanTrueHeadingDegrees;
  msgPtr->gyroscope_x_velocity_millidegreesps =  busPtr->GyroscopeXVelocityMillidegreesps;
  msgPtr->gyroscope_y_velocity_millidegreesps =  busPtr->GyroscopeYVelocityMillidegreesps;
  msgPtr->gyroscope_z_velocity_millidegreesps =  busPtr->GyroscopeZVelocityMillidegreesps;
  msgPtr->rudder_port_angle_degrees =  busPtr->RudderPortAngleDegrees;
  msgPtr->rudder_stbd_angle_degrees =  busPtr->RudderStbdAngleDegrees;
  msgPtr->sailencoder_degrees =  busPtr->SailencoderDegrees;
  msgPtr->winch_jib_angle_degrees =  busPtr->WinchJibAngleDegrees;
  msgPtr->winch_main_angle_degrees =  busPtr->WinchMainAngleDegrees;
  msgPtr->wind_sensor_1_angle_degrees =  busPtr->WindSensor1AngleDegrees;
  msgPtr->wind_sensor_1_speed_knots =  busPtr->WindSensor1SpeedKnots;
  msgPtr->wind_sensor_2_angle_degrees =  busPtr->WindSensor2AngleDegrees;
  msgPtr->wind_sensor_2_speed_knots =  busPtr->WindSensor2SpeedKnots;
  msgPtr->wind_sensor_3_angle_degrees =  busPtr->WindSensor3AngleDegrees;
  msgPtr->wind_sensor_3_speed_knots =  busPtr->WindSensor3SpeedKnots;
}

void convertToBus(SL_Bus_boat_controller_sailbot_msg_Sensors* busPtr, sailbot_msg::Sensors const* msgPtr)
{
  const std::string rosMessageType("sailbot_msg/Sensors");

  busPtr->AccelerometerXForceMillig =  msgPtr->accelerometer_x_force_millig;
  busPtr->AccelerometerYForceMillig =  msgPtr->accelerometer_y_force_millig;
  busPtr->AccelerometerZForceMillig =  msgPtr->accelerometer_z_force_millig;
  busPtr->GpsAisGroundspeedKnots =  msgPtr->gps_ais_groundspeed_knots;
  busPtr->GpsAisLatitudeDegreeMinutes =  msgPtr->gps_ais_latitude_degreeMinutes;
  busPtr->GpsAisLongitudeDegreeMinutes =  msgPtr->gps_ais_longitude_degreeMinutes;
  busPtr->GpsAisMagneticVariationDegrees =  msgPtr->gps_ais_magnetic_variation_degrees;
  busPtr->GpsAisState =  msgPtr->gps_ais_state;
  convertToBusVariablePrimitiveArray(busPtr->GpsAisTimestampUtc, busPtr->GpsAisTimestampUtc_SL_Info, msgPtr->gps_ais_timestamp_utc, slros::EnabledWarning(rosMessageType, "gps_ais_timestamp_utc"));
  busPtr->GpsAisTrackMadeGoodDegrees =  msgPtr->gps_ais_track_made_good_degrees;
  busPtr->GpsAisTrueHeadingDegrees =  msgPtr->gps_ais_true_heading_degrees;
  busPtr->GpsCanGroundspeedKnots =  msgPtr->gps_can_groundspeed_knots;
  busPtr->GpsCanLatitudeDegreeMinutes =  msgPtr->gps_can_latitude_degreeMinutes;
  busPtr->GpsCanLongitudeDegreeMinutes =  msgPtr->gps_can_longitude_degreeMinutes;
  busPtr->GpsCanMagneticVariationDegrees =  msgPtr->gps_can_magnetic_variation_degrees;
  busPtr->GpsCanState =  msgPtr->gps_can_state;
  convertToBusVariablePrimitiveArray(busPtr->GpsCanTimestampUtc, busPtr->GpsCanTimestampUtc_SL_Info, msgPtr->gps_can_timestamp_utc, slros::EnabledWarning(rosMessageType, "gps_can_timestamp_utc"));
  busPtr->GpsCanTrackMadeGoodDegrees =  msgPtr->gps_can_track_made_good_degrees;
  busPtr->GpsCanTrueHeadingDegrees =  msgPtr->gps_can_true_heading_degrees;
  busPtr->GyroscopeXVelocityMillidegreesps =  msgPtr->gyroscope_x_velocity_millidegreesps;
  busPtr->GyroscopeYVelocityMillidegreesps =  msgPtr->gyroscope_y_velocity_millidegreesps;
  busPtr->GyroscopeZVelocityMillidegreesps =  msgPtr->gyroscope_z_velocity_millidegreesps;
  busPtr->RudderPortAngleDegrees =  msgPtr->rudder_port_angle_degrees;
  busPtr->RudderStbdAngleDegrees =  msgPtr->rudder_stbd_angle_degrees;
  busPtr->SailencoderDegrees =  msgPtr->sailencoder_degrees;
  busPtr->WinchJibAngleDegrees =  msgPtr->winch_jib_angle_degrees;
  busPtr->WinchMainAngleDegrees =  msgPtr->winch_main_angle_degrees;
  busPtr->WindSensor1AngleDegrees =  msgPtr->wind_sensor_1_angle_degrees;
  busPtr->WindSensor1SpeedKnots =  msgPtr->wind_sensor_1_speed_knots;
  busPtr->WindSensor2AngleDegrees =  msgPtr->wind_sensor_2_angle_degrees;
  busPtr->WindSensor2SpeedKnots =  msgPtr->wind_sensor_2_speed_knots;
  busPtr->WindSensor3AngleDegrees =  msgPtr->wind_sensor_3_angle_degrees;
  busPtr->WindSensor3SpeedKnots =  msgPtr->wind_sensor_3_speed_knots;
}


// Conversions between SL_Bus_boat_controller_sailbot_msg_actuation_angle and sailbot_msg::actuation_angle

void convertFromBus(sailbot_msg::actuation_angle* msgPtr, SL_Bus_boat_controller_sailbot_msg_actuation_angle const* busPtr)
{
  const std::string rosMessageType("sailbot_msg/actuation_angle");

  msgPtr->abs_sail_angle_degrees =  busPtr->AbsSailAngleDegrees;
  msgPtr->rudder_angle_degrees =  busPtr->RudderAngleDegrees;
}

void convertToBus(SL_Bus_boat_controller_sailbot_msg_actuation_angle* busPtr, sailbot_msg::actuation_angle const* msgPtr)
{
  const std::string rosMessageType("sailbot_msg/actuation_angle");

  busPtr->AbsSailAngleDegrees =  msgPtr->abs_sail_angle_degrees;
  busPtr->RudderAngleDegrees =  msgPtr->rudder_angle_degrees;
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

