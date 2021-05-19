//
//  boat_controller_dt.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "boat_controller".
//
//  Model version              : 1.62
//  Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
//  C++ source code generated on : Tue May 18 19:32:54 2021
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Intel->x86-64 (Linux 64)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(SL_Bus_boat_controller_sailbot_msg_actuation_angle),
  sizeof(SL_Bus_boat_controller_sailbot_msg_heading),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_boat_controller_sailbot_msg_Sensors),
  sizeof(ros_slros_internal_block_Publ_T),
  sizeof(ros_slros_internal_block_Subs_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "SL_Bus_boat_controller_sailbot_msg_actuation_angle",
  "SL_Bus_boat_controller_sailbot_msg_heading",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_boat_controller_sailbot_msg_Sensors",
  "ros_slros_internal_block_Publ_T",
  "ros_slros_internal_block_Subs_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&boat_controller_B.rudderanglesaturation), 0, 0, 5 }
  ,

  { (char_T *)(&boat_controller_DW.obj), 18, 0, 1 },

  { (char_T *)(&boat_controller_DW.obj_i), 19, 0, 2 },

  { (char_T *)(&boat_controller_DW.ToWorkspace_PWORK.LoggedData), 11, 0, 3 },

  { (char_T *)(&boat_controller_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 4 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  5U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&boat_controller_P.PIDController_P), 0, 0, 1 },

  { (char_T *)(&boat_controller_P.Out1_Y0), 17, 0, 1 },

  { (char_T *)(&boat_controller_P.Constant_Value), 17, 0, 1 },

  { (char_T *)(&boat_controller_P.Constant_Value_f), 14, 0, 1 },

  { (char_T *)(&boat_controller_P.Out1_Y0_f), 15, 0, 1 },

  { (char_T *)(&boat_controller_P.Constant_Value_m), 15, 0, 1 },

  { (char_T *)(&boat_controller_P.WindSensor1SpeedMetersPerSec_Y0), 0, 0, 13 },

  { (char_T *)(&boat_controller_P.Gain1_Gain_n), 6, 0, 1 },

  { (char_T *)(&boat_controller_P.Gain1_Gain_l), 1, 0, 1 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  9U,
  rtPTransitions
};

// [EOF] boat_controller_dt.h
