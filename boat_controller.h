//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller.h
//
// Code generated for Simulink model 'boat_controller'.
//
// Model version                  : 1.63
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun Jun 13 17:43:40 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_boat_controller_h_
#define RTW_HEADER_boat_controller_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros_initialize.h"
#include "boat_controller_types.h"

// Shared type includes
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
# define rtmGetRTWExtModeInfo(rtm)     ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

// Block signals (default storage)
typedef struct {
  real_T Cls[101];
  real_T Cds[101];
  real_T c_x_data[101];
  real_T b[101];
  SL_Bus_boat_controller_sailbot_msg_Sensors b_varargout_2;
  SL_Bus_boat_controller_sailbot_msg_actuation_angle BusAssignment;// '<Root>/Bus Assignment' 
  real_T rudderanglesaturation;        // '<S9>/rudder angle saturation'
  real_T error;                        // '<S9>/MATLAB Function'
  real_T Gain1;                        // '<S12>/Gain1'
  real_T CastToDouble1;                // '<S2>/Cast To Double1'
  real_T CastToDouble2;                // '<S2>/Cast To Double2'
  real_T q;
  real_T sailAngle;                    // '<S9>/MATLAB Function2'
  SL_Bus_boat_controller_sailbot_msg_heading b_varargout_2_m;
} B_boat_controller_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_Publ_T obj; // '<S4>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_i;// '<S8>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_p;// '<S7>/SourceBlock'
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 // '<S9>/To Workspace'

  struct {
    void *LoggedData;
  } delta_rt_PWORK;                    // '<S9>/delta_r(t)'

  struct {
    void *LoggedData;
  } headingErrorRadianst_PWORK;        // '<S9>/headingErrorRadians (t)'

  int8_T EnabledSubsystem_SubsysRanBC; // '<S8>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_b;// '<S7>/Enabled Subsystem'
  int8_T EnabledSubsystem1_SubsysRanBC;// '<Root>/Enabled Subsystem1'
  int8_T EnabledSubsystem_SubsysRanBC_i;// '<Root>/Enabled Subsystem'
} DW_boat_controller_T;

// Parameters (default storage)
struct P_boat_controller_T_ {
  real_T PIDController_P;              // Mask Parameter: PIDController_P
                                          //  Referenced by: '<S56>/Proportional Gain'

  SL_Bus_boat_controller_sailbot_msg_Sensors Out1_Y0;// Computed Parameter: Out1_Y0
                                                        //  Referenced by: '<S14>/Out1'

  SL_Bus_boat_controller_sailbot_msg_Sensors Constant_Value;// Computed Parameter: Constant_Value
                                                               //  Referenced by: '<S8>/Constant'

  SL_Bus_boat_controller_sailbot_msg_actuation_angle Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_boat_controller_sailbot_msg_heading Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                          //  Referenced by: '<S13>/Out1'

  SL_Bus_boat_controller_sailbot_msg_heading Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                                 //  Referenced by: '<S7>/Constant'

  real_T WindSensor1SpeedMetersPerSec_Y0;
                          // Computed Parameter: WindSensor1SpeedMetersPerSec_Y0
                             //  Referenced by: '<S2>/WindSensor1SpeedMetersPerSec'

  real_T GpsTrueHeadingRad_Y0;       // Computed Parameter: GpsTrueHeadingRad_Y0
                                        //  Referenced by: '<S2>/GpsTrueHeadingRad'

  real_T WindSensor1DirectionRad_Y0;
                               // Computed Parameter: WindSensor1DirectionRad_Y0
                                  //  Referenced by: '<S2>/WindSensor1DirectionRad'

  real_T desiredHeadingRad_Y0;       // Computed Parameter: desiredHeadingRad_Y0
                                        //  Referenced by: '<S3>/desiredHeadingRad'

  real_T Gain1_Gain;                   // Expression: pi/180
                                          //  Referenced by: '<S12>/Gain1'

  real_T rudderanglesaturation_UpperSat;// Expression: pi/3
                                           //  Referenced by: '<S9>/rudder angle saturation'

  real_T rudderanglesaturation_LowerSat;// Expression: -pi/3
                                           //  Referenced by: '<S9>/rudder angle saturation'

  real_T Saturation_UpperSat;          // Expression: pi/3
                                          //  Referenced by: '<Root>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -pi/3
                                          //  Referenced by: '<Root>/Saturation'

  real_T Gain_Gain;                    // Expression: 180/pi
                                          //  Referenced by: '<S6>/Gain'

  real_T Saturation1_UpperSat;         // Expression: pi/2
                                          //  Referenced by: '<Root>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: 0
                                          //  Referenced by: '<Root>/Saturation1'

  real_T Gain_Gain_b;                  // Expression: 180/pi
                                          //  Referenced by: '<S5>/Gain'

  int32_T Gain1_Gain_n;                // Computed Parameter: Gain1_Gain_n
                                          //  Referenced by: '<S10>/Gain1'

  real32_T Gain1_Gain_l;               // Computed Parameter: Gain1_Gain_l
                                          //  Referenced by: '<S11>/Gain1'

};

// Real-time Model Data Structure
struct tag_RTM_boat_controller_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_boat_controller_T boat_controller_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_boat_controller_T boat_controller_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_boat_controller_T boat_controller_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void boat_controller_initialize(void);
  extern void boat_controller_step(void);
  extern void boat_controller_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_boat_controller_T *const boat_controller_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/Cast To Double3' : Unused code path elimination
//  Block '<S2>/Knots2MertersPerSec' : Unused code path elimination
//  Block '<S15>/Assertion' : Unused code path elimination
//  Block '<S15>/conjunction' : Unused code path elimination
//  Block '<S15>/max_relop' : Unused code path elimination
//  Block '<S15>/min_relop' : Unused code path elimination
//  Block '<S19>/FromWs' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'boat_controller'
//  '<S1>'   : 'boat_controller/Blank Message'
//  '<S2>'   : 'boat_controller/Enabled Subsystem'
//  '<S3>'   : 'boat_controller/Enabled Subsystem1'
//  '<S4>'   : 'boat_controller/Publish'
//  '<S5>'   : 'boat_controller/Radians to Degrees1'
//  '<S6>'   : 'boat_controller/Radians to Degrees2'
//  '<S7>'   : 'boat_controller/Subscribe1'
//  '<S8>'   : 'boat_controller/Subscribe2'
//  '<S9>'   : 'boat_controller/controller using apparent wind angle'
//  '<S10>'  : 'boat_controller/Enabled Subsystem/Degrees to Radians'
//  '<S11>'  : 'boat_controller/Enabled Subsystem/Degrees to Radians1'
//  '<S12>'  : 'boat_controller/Enabled Subsystem1/Degrees to Radians'
//  '<S13>'  : 'boat_controller/Subscribe1/Enabled Subsystem'
//  '<S14>'  : 'boat_controller/Subscribe2/Enabled Subsystem'
//  '<S15>'  : 'boat_controller/controller using apparent wind angle/Check  Dynamic Range'
//  '<S16>'  : 'boat_controller/controller using apparent wind angle/MATLAB Function'
//  '<S17>'  : 'boat_controller/controller using apparent wind angle/MATLAB Function2'
//  '<S18>'  : 'boat_controller/controller using apparent wind angle/PID Controller'
//  '<S19>'  : 'boat_controller/controller using apparent wind angle/Signal Builder'
//  '<S20>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Anti-windup'
//  '<S21>'  : 'boat_controller/controller using apparent wind angle/PID Controller/D Gain'
//  '<S22>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Filter'
//  '<S23>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Filter ICs'
//  '<S24>'  : 'boat_controller/controller using apparent wind angle/PID Controller/I Gain'
//  '<S25>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Ideal P Gain'
//  '<S26>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Ideal P Gain Fdbk'
//  '<S27>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Integrator'
//  '<S28>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Integrator ICs'
//  '<S29>'  : 'boat_controller/controller using apparent wind angle/PID Controller/N Copy'
//  '<S30>'  : 'boat_controller/controller using apparent wind angle/PID Controller/N Gain'
//  '<S31>'  : 'boat_controller/controller using apparent wind angle/PID Controller/P Copy'
//  '<S32>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Parallel P Gain'
//  '<S33>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Reset Signal'
//  '<S34>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Saturation'
//  '<S35>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Saturation Fdbk'
//  '<S36>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Sum'
//  '<S37>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Sum Fdbk'
//  '<S38>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tracking Mode'
//  '<S39>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tracking Mode Sum'
//  '<S40>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tsamp - Integral'
//  '<S41>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tsamp - Ngain'
//  '<S42>'  : 'boat_controller/controller using apparent wind angle/PID Controller/postSat Signal'
//  '<S43>'  : 'boat_controller/controller using apparent wind angle/PID Controller/preSat Signal'
//  '<S44>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Anti-windup/Disabled'
//  '<S45>'  : 'boat_controller/controller using apparent wind angle/PID Controller/D Gain/Disabled'
//  '<S46>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Filter/Disabled'
//  '<S47>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Filter ICs/Disabled'
//  '<S48>'  : 'boat_controller/controller using apparent wind angle/PID Controller/I Gain/Disabled'
//  '<S49>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Ideal P Gain/Passthrough'
//  '<S50>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S51>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Integrator/Disabled'
//  '<S52>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Integrator ICs/Disabled'
//  '<S53>'  : 'boat_controller/controller using apparent wind angle/PID Controller/N Copy/Disabled wSignal Specification'
//  '<S54>'  : 'boat_controller/controller using apparent wind angle/PID Controller/N Gain/Disabled'
//  '<S55>'  : 'boat_controller/controller using apparent wind angle/PID Controller/P Copy/Disabled'
//  '<S56>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S57>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Reset Signal/Disabled'
//  '<S58>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Saturation/Passthrough'
//  '<S59>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Saturation Fdbk/Disabled'
//  '<S60>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Sum/Passthrough_P'
//  '<S61>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Sum Fdbk/Disabled'
//  '<S62>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tracking Mode/Disabled'
//  '<S63>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S64>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tsamp - Integral/Disabled wSignal Specification'
//  '<S65>'  : 'boat_controller/controller using apparent wind angle/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S66>'  : 'boat_controller/controller using apparent wind angle/PID Controller/postSat Signal/Forward_Path'
//  '<S67>'  : 'boat_controller/controller using apparent wind angle/PID Controller/preSat Signal/Forward_Path'

#endif                                 // RTW_HEADER_boat_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
