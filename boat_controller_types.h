//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller_types.h
//
// Code generated for Simulink model 'boat_controller'.
//
// Model version                  : 1.55
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Fri Mar 19 13:01:20 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_boat_controller_types_h_
#define RTW_HEADER_boat_controller_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_actuation_angle_
#define DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_actuation_angle_

// MsgType=sailbot_msg/actuation_angle
typedef struct {
  real_T Rudder;
  real_T Winch;
} SL_Bus_boat_controller_sailbot_msg_actuation_angle;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_Sensors_
#define DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_Sensors_

// MsgType=sailbot_msg/Sensors
typedef struct {
  int32_T BoomAngleSensorAngle;
  int32_T WindSensor0Speed;
  int32_T WindSensor0Direction;
  int32_T WindSensor0Reference;
  int32_T WindSensor1Speed;
  int32_T WindSensor1Direction;
  int32_T WindSensor1Reference;
  int32_T WindSensor2Speed;
  int32_T WindSensor2Direction;
  int32_T WindSensor2Reference;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Gps0Timestamp_SL_Info:TruncateAction=warn 
  uint8_T Gps0Timestamp[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Gps0Timestamp
  SL_Bus_ROSVariableLengthArrayInfo Gps0Timestamp_SL_Info;
  real_T Gps0Latitude;
  real_T Gps0Longitude;
  boolean_T Gps0LatitudeLoc;
  boolean_T Gps0LongitudeLoc;
  int32_T Gps0Groundspeed;
  int32_T Gps0TrackMadeGood;
  int32_T Gps0TrueHeading;
  int32_T Gps0MagneticVariation;
  boolean_T Gps0MagneticVariationSense;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Gps1Timestamp_SL_Info:TruncateAction=warn 
  uint8_T Gps1Timestamp[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Gps1Timestamp
  SL_Bus_ROSVariableLengthArrayInfo Gps1Timestamp_SL_Info;
  real_T Gps1Latitude;
  real_T Gps1Longitude;
  boolean_T Gps1LatitudeLoc;
  boolean_T Gps1LongitudeLoc;
  int32_T Gps1Groundspeed;
  int32_T Gps1TrackMadeGood;
  int32_T Gps1TrueHeading;
  int32_T Gps1MagneticVariation;
  boolean_T Gps1MagneticVariationSense;
  int32_T AccelerometerXAxisAcceleration;
  int32_T AccelerometerYAxisAcceleration;
  int32_T AccelerometerZAxisAcceleration;
} SL_Bus_boat_controller_sailbot_msg_Sensors;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_heading_
#define DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_heading_

// MsgType=sailbot_msg/heading
typedef struct {
  real_T HeadingDegrees;
} SL_Bus_boat_controller_sailbot_msg_heading;

#endif

#ifndef struct_tag_rkSooZHJZnr3Dpfu1LNqfH
#define struct_tag_rkSooZHJZnr3Dpfu1LNqfH

struct tag_rkSooZHJZnr3Dpfu1LNqfH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_rkSooZHJZnr3Dpfu1LNqfH

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct tag_rkSooZHJZnr3Dpfu1LNqfH ros_slros_internal_block_Publ_T;

#endif                                 //typedef_ros_slros_internal_block_Publ_T

#ifndef struct_tag_9SewJ4y3IXNs5GrZti8qkG
#define struct_tag_9SewJ4y3IXNs5GrZti8qkG

struct tag_9SewJ4y3IXNs5GrZti8qkG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_9SewJ4y3IXNs5GrZti8qkG

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct tag_9SewJ4y3IXNs5GrZti8qkG ros_slros_internal_block_Subs_T;

#endif                                 //typedef_ros_slros_internal_block_Subs_T

// Parameters (default storage)
typedef struct P_boat_controller_T_ P_boat_controller_T;

// Forward declaration for rtModel
typedef struct tag_RTM_boat_controller_T RT_MODEL_boat_controller_T;

#endif                                 // RTW_HEADER_boat_controller_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
