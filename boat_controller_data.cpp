//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller_data.cpp
//
// Code generated for Simulink model 'boat_controller'.
//
// Model version                  : 1.54
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Fri Mar 19 12:30:06 2021
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
  //  Referenced by: '<S54>/Proportional Gain'

  0.5,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S11>/Out1'

  {
    0,                                 // BoomAngleSensorAngle
    0,                                 // WindSensor0Speed
    0,                                 // WindSensor0Direction
    0,                                 // WindSensor0Reference
    0,                                 // WindSensor1Speed
    0,                                 // WindSensor1Direction
    0,                                 // WindSensor1Reference
    0,                                 // WindSensor2Speed
    0,                                 // WindSensor2Direction
    0,                                 // WindSensor2Reference

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Gps0Timestamp

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Gps0Timestamp_SL_Info
    0.0,                               // Gps0Latitude
    0.0,                               // Gps0Longitude
    false,                             // Gps0LatitudeLoc
    false,                             // Gps0LongitudeLoc
    0,                                 // Gps0Groundspeed
    0,                                 // Gps0TrackMadeGood
    0,                                 // Gps0TrueHeading
    0,                                 // Gps0MagneticVariation
    false,                             // Gps0MagneticVariationSense

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Gps1Timestamp

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Gps1Timestamp_SL_Info
    0.0,                               // Gps1Latitude
    0.0,                               // Gps1Longitude
    false,                             // Gps1LatitudeLoc
    false,                             // Gps1LongitudeLoc
    0,                                 // Gps1Groundspeed
    0,                                 // Gps1TrackMadeGood
    0,                                 // Gps1TrueHeading
    0,                                 // Gps1MagneticVariation
    false,                             // Gps1MagneticVariationSense
    0,                                 // AccelerometerXAxisAcceleration
    0,                                 // AccelerometerYAxisAcceleration
    0                                  // AccelerometerZAxisAcceleration
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S5>/Constant'

  {
    0,                                 // BoomAngleSensorAngle
    0,                                 // WindSensor0Speed
    0,                                 // WindSensor0Direction
    0,                                 // WindSensor0Reference
    0,                                 // WindSensor1Speed
    0,                                 // WindSensor1Direction
    0,                                 // WindSensor1Reference
    0,                                 // WindSensor2Speed
    0,                                 // WindSensor2Direction
    0,                                 // WindSensor2Reference

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Gps0Timestamp

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Gps0Timestamp_SL_Info
    0.0,                               // Gps0Latitude
    0.0,                               // Gps0Longitude
    false,                             // Gps0LatitudeLoc
    false,                             // Gps0LongitudeLoc
    0,                                 // Gps0Groundspeed
    0,                                 // Gps0TrackMadeGood
    0,                                 // Gps0TrueHeading
    0,                                 // Gps0MagneticVariation
    false,                             // Gps0MagneticVariationSense

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Gps1Timestamp

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Gps1Timestamp_SL_Info
    0.0,                               // Gps1Latitude
    0.0,                               // Gps1Longitude
    false,                             // Gps1LatitudeLoc
    false,                             // Gps1LongitudeLoc
    0,                                 // Gps1Groundspeed
    0,                                 // Gps1TrackMadeGood
    0,                                 // Gps1TrueHeading
    0,                                 // Gps1MagneticVariation
    false,                             // Gps1MagneticVariationSense
    0,                                 // AccelerometerXAxisAcceleration
    0,                                 // AccelerometerYAxisAcceleration
    0                                  // AccelerometerZAxisAcceleration
  },

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S1>/Constant'

  {
    0.0,                               // Rudder
    0.0                                // Winch
  },

  // Computed Parameter: Out1_Y0_f
  //  Referenced by: '<S12>/Out1'

  {
    0.0                                // HeadingDegrees
  },

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S6>/Constant'

  {
    0.0                                // HeadingDegrees
  },

  // Computed Parameter: WindSensor0SpeedMetersPerSec_Y0
  //  Referenced by: '<S2>/WindSensor0SpeedMetersPerSec'

  0.0,

  // Computed Parameter: Gps0TrueHeadingRad_Y0
  //  Referenced by: '<S2>/Gps0TrueHeadingRad'

  0.0,

  // Computed Parameter: WindSensor0DirectionRad_Y0
  //  Referenced by: '<S2>/WindSensor0DirectionRad'

  0.0,

  // Computed Parameter: desiredHeadingRad_Y0
  //  Referenced by: '<S3>/desiredHeadingRad'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S10>/Gain1'

  0.017453292519943295,

  // Expression: pi/3
  //  Referenced by: '<S7>/rudder angle saturation'

  1.0471975511965976,

  // Expression: -pi/3
  //  Referenced by: '<S7>/rudder angle saturation'

  -1.0471975511965976,

  // Expression: pi/3
  //  Referenced by: '<Root>/Saturation'

  1.0471975511965976,

  // Expression: -pi/3
  //  Referenced by: '<Root>/Saturation'

  -1.0471975511965976,

  // Expression: pi/2
  //  Referenced by: '<Root>/Saturation1'

  1.5707963267948966,

  // Expression: 0
  //  Referenced by: '<Root>/Saturation1'

  0.0,

  // Computed Parameter: Gain1_Gain_l
  //  Referenced by: '<S9>/Gain1'

  0,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S8>/Gain1'

  0
};

//
// File trailer for generated code.
//
// [EOF]
//
