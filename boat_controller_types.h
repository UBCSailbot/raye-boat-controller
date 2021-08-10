//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller_types.h
//
// Code generated for Simulink model 'boat_controller'.
//
// Model version                  : 1.63
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue Aug 10 16:45:42 2021
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
  real_T RudderAngleDegrees;
  real_T AbsSailAngleDegrees;
} SL_Bus_boat_controller_sailbot_msg_actuation_angle;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_heading_
#define DEFINED_TYPEDEF_FOR_SL_Bus_boat_controller_sailbot_msg_heading_

// MsgType=sailbot_msg/heading
typedef struct {
  real_T HeadingDegrees;
} SL_Bus_boat_controller_sailbot_msg_heading;

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
  int32_T SailencoderDegrees;
  real32_T WindSensor1SpeedKnots;
  int32_T WindSensor1AngleDegrees;
  real32_T WindSensor2SpeedKnots;
  int32_T WindSensor2AngleDegrees;
  real32_T WindSensor3SpeedKnots;
  int32_T WindSensor3AngleDegrees;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=GpsCanTimestampUtc_SL_Info:TruncateAction=warn 
  uint8_T GpsCanTimestampUtc[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=GpsCanTimestampUtc
  SL_Bus_ROSVariableLengthArrayInfo GpsCanTimestampUtc_SL_Info;
  real32_T GpsCanLatitudeDegrees;
  real32_T GpsCanLongitudeDegrees;
  real32_T GpsCanGroundspeedKnots;
  real32_T GpsCanTrackMadeGoodDegrees;
  real32_T GpsCanTrueHeadingDegrees;
  real32_T GpsCanMagneticVariationDegrees;
  boolean_T GpsCanState;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=GpsAisTimestampUtc_SL_Info:TruncateAction=warn 
  uint8_T GpsAisTimestampUtc[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=GpsAisTimestampUtc
  SL_Bus_ROSVariableLengthArrayInfo GpsAisTimestampUtc_SL_Info;
  real32_T GpsAisLatitudeDegrees;
  real32_T GpsAisLongitudeDegrees;
  real32_T GpsAisGroundspeedKnots;
  real32_T GpsAisTrackMadeGoodDegrees;
  real32_T GpsAisTrueHeadingDegrees;
  real32_T GpsAisMagneticVariationDegrees;
  boolean_T GpsAisState;
  int32_T WinchMainAngleDegrees;
  int32_T WinchJibAngleDegrees;
  real32_T RudderPortAngleDegrees;
  real32_T RudderStbdAngleDegrees;
  real32_T AccelerometerXForceMillig;
  real32_T AccelerometerYForceMillig;
  real32_T AccelerometerZForceMillig;
  real32_T GyroscopeXVelocityMillidegreesps;
  real32_T GyroscopeYVelocityMillidegreesps;
  real32_T GyroscopeZVelocityMillidegreesps;
} SL_Bus_boat_controller_sailbot_msg_Sensors;

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
