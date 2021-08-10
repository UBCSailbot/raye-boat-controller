//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller_data.cpp
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
#include "boat_controller.h"
#include "boat_controller_private.h"

// Block parameters (default storage)
P_boat_controller_T boat_controller_P = {
  // Mask Parameter: PIDController_P
  //  Referenced by: '<S56>/Proportional Gain'

  0.5,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S14>/Out1'

  {
    0,                                 // SailencoderDegrees
    0.0F,                              // WindSensor1SpeedKnots
    0,                                 // WindSensor1AngleDegrees
    0.0F,                              // WindSensor2SpeedKnots
    0,                                 // WindSensor2AngleDegrees
    0.0F,                              // WindSensor3SpeedKnots
    0,                                 // WindSensor3AngleDegrees

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // GpsCanTimestampUtc

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // GpsCanTimestampUtc_SL_Info
    0.0F,                              // GpsCanLatitudeDegrees
    0.0F,                              // GpsCanLongitudeDegrees
    0.0F,                              // GpsCanGroundspeedKnots
    0.0F,                              // GpsCanTrackMadeGoodDegrees
    0.0F,                              // GpsCanTrueHeadingDegrees
    0.0F,                              // GpsCanMagneticVariationDegrees
    false,                             // GpsCanState

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // GpsAisTimestampUtc

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // GpsAisTimestampUtc_SL_Info
    0.0F,                              // GpsAisLatitudeDegrees
    0.0F,                              // GpsAisLongitudeDegrees
    0.0F,                              // GpsAisGroundspeedKnots
    0.0F,                              // GpsAisTrackMadeGoodDegrees
    0.0F,                              // GpsAisTrueHeadingDegrees
    0.0F,                              // GpsAisMagneticVariationDegrees
    false,                             // GpsAisState
    0,                                 // WinchMainAngleDegrees
    0,                                 // WinchJibAngleDegrees
    0.0F,                              // RudderPortAngleDegrees
    0.0F,                              // RudderStbdAngleDegrees
    0.0F,                              // AccelerometerXForceMillig
    0.0F,                              // AccelerometerYForceMillig
    0.0F,                              // AccelerometerZForceMillig
    0.0F,                              // GyroscopeXVelocityMillidegreesps
    0.0F,                              // GyroscopeYVelocityMillidegreesps
    0.0F                               // GyroscopeZVelocityMillidegreesps
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S8>/Constant'

  {
    0,                                 // SailencoderDegrees
    0.0F,                              // WindSensor1SpeedKnots
    0,                                 // WindSensor1AngleDegrees
    0.0F,                              // WindSensor2SpeedKnots
    0,                                 // WindSensor2AngleDegrees
    0.0F,                              // WindSensor3SpeedKnots
    0,                                 // WindSensor3AngleDegrees

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // GpsCanTimestampUtc

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // GpsCanTimestampUtc_SL_Info
    0.0F,                              // GpsCanLatitudeDegrees
    0.0F,                              // GpsCanLongitudeDegrees
    0.0F,                              // GpsCanGroundspeedKnots
    0.0F,                              // GpsCanTrackMadeGoodDegrees
    0.0F,                              // GpsCanTrueHeadingDegrees
    0.0F,                              // GpsCanMagneticVariationDegrees
    false,                             // GpsCanState

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // GpsAisTimestampUtc

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // GpsAisTimestampUtc_SL_Info
    0.0F,                              // GpsAisLatitudeDegrees
    0.0F,                              // GpsAisLongitudeDegrees
    0.0F,                              // GpsAisGroundspeedKnots
    0.0F,                              // GpsAisTrackMadeGoodDegrees
    0.0F,                              // GpsAisTrueHeadingDegrees
    0.0F,                              // GpsAisMagneticVariationDegrees
    false,                             // GpsAisState
    0,                                 // WinchMainAngleDegrees
    0,                                 // WinchJibAngleDegrees
    0.0F,                              // RudderPortAngleDegrees
    0.0F,                              // RudderStbdAngleDegrees
    0.0F,                              // AccelerometerXForceMillig
    0.0F,                              // AccelerometerYForceMillig
    0.0F,                              // AccelerometerZForceMillig
    0.0F,                              // GyroscopeXVelocityMillidegreesps
    0.0F,                              // GyroscopeYVelocityMillidegreesps
    0.0F                               // GyroscopeZVelocityMillidegreesps
  },

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S1>/Constant'

  {
    0.0,                               // RudderAngleDegrees
    0.0                                // AbsSailAngleDegrees
  },

  // Computed Parameter: Out1_Y0_f
  //  Referenced by: '<S13>/Out1'

  {
    0.0                                // HeadingDegrees
  },

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S7>/Constant'

  {
    0.0                                // HeadingDegrees
  },

  // Computed Parameter: WindSensor1SpeedMetersPerSec_Y0
  //  Referenced by: '<S2>/WindSensor1SpeedMetersPerSec'

  0.0,

  // Computed Parameter: GpsTrueHeadingRad_Y0
  //  Referenced by: '<S2>/GpsTrueHeadingRad'

  0.0,

  // Computed Parameter: WindSensor1DirectionRad_Y0
  //  Referenced by: '<S2>/WindSensor1DirectionRad'

  0.0,

  // Computed Parameter: desiredHeadingRad_Y0
  //  Referenced by: '<S3>/desiredHeadingRad'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S12>/Gain1'

  0.017453292519943295,

  // Expression: pi/3
  //  Referenced by: '<S9>/rudder angle saturation'

  1.0471975511965976,

  // Expression: -pi/3
  //  Referenced by: '<S9>/rudder angle saturation'

  -1.0471975511965976,

  // Expression: pi/3
  //  Referenced by: '<Root>/Saturation'

  1.0471975511965976,

  // Expression: -pi/3
  //  Referenced by: '<Root>/Saturation'

  -1.0471975511965976,

  // Expression: 180/pi
  //  Referenced by: '<S6>/Gain'

  57.295779513082323,

  // Expression: pi/2
  //  Referenced by: '<Root>/Saturation1'

  1.5707963267948966,

  // Expression: 0
  //  Referenced by: '<Root>/Saturation1'

  0.0,

  // Expression: 180/pi
  //  Referenced by: '<S5>/Gain'

  57.295779513082323,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S10>/Gain1'

  0,

  // Computed Parameter: Gain1_Gain_l
  //  Referenced by: '<S11>/Gain1'

  0.0174532924F
};

//
// File trailer for generated code.
//
// [EOF]
//
