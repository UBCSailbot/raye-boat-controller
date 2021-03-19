//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: boat_controller.cpp
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
#include "boat_controller.h"
#include "boat_controller_private.h"
#include "boat_controller_dt.h"

// Block signals (default storage)
B_boat_controller_T boat_controller_B;

// Block states (default storage)
DW_boat_controller_T boat_controller_DW;

// Real-time model
RT_MODEL_boat_controller_T boat_controller_M_ = RT_MODEL_boat_controller_T();
RT_MODEL_boat_controller_T *const boat_controller_M = &boat_controller_M_;

// Forward declaration for local functions
static real_T boat_controller_ppval(const real_T pp_breaks[73], const real_T
  pp_coefs[288], real_T x);
static void boat_controller_sailcoef(real_T attack[101], real_T Cls[101], real_T
  Cds[101]);
static void matlabCodegenHandle_matlabCod_m(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);

// Function for MATLAB Function: '<S7>/MATLAB Function2'
static real_T boat_controller_ppval(const real_T pp_breaks[73], const real_T
  pp_coefs[288], real_T x)
{
  real_T v;
  real_T xloc;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  if (rtIsNaN(x)) {
    v = x;
  } else {
    low_i = 0;
    low_ip1 = 2;
    high_i = 73;
    while (high_i > low_ip1) {
      mid_i = static_cast<int32_T>(static_cast<int32_T>(static_cast<int32_T>
        (low_i + high_i) + 1) >> 1);
      if (x >= pp_breaks[static_cast<int32_T>(mid_i - 1)]) {
        low_i = static_cast<int32_T>(mid_i - 1);
        low_ip1 = static_cast<int32_T>(mid_i + 1);
      } else {
        high_i = mid_i;
      }
    }

    xloc = x - pp_breaks[low_i];
    v = ((xloc * pp_coefs[low_i] + pp_coefs[static_cast<int32_T>(low_i + 72)]) *
         xloc + pp_coefs[static_cast<int32_T>(low_i + 144)]) * xloc + pp_coefs[
      static_cast<int32_T>(low_i + 216)];
  }

  return v;
}

// Function for MATLAB Function: '<S7>/MATLAB Function2'
static void boat_controller_sailcoef(real_T attack[101], real_T Cls[101], real_T
  Cds[101])
{
  int32_T k;
  real_T attack_0;
  static const real_T b_breaks[73] = { -180.0, -175.0, -170.0, -165.0, -160.0,
    -155.0, -150.0, -145.0, -140.0, -135.0, -130.0, -125.0, -120.0,
    -114.99999999999999, -110.00000000000001, -105.0, -100.0, -95.0, -90.0,
    -85.0, -80.0, -75.0, -70.0, -65.0, -60.0, -55.000000000000007, -50.0, -45.0,
    -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0,
    20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.000000000000007, 60.0, 65.0,
    70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0, 110.00000000000001,
    114.99999999999999, 120.0, 125.0, 130.0, 135.0, 140.0, 145.0, 150.0, 155.0,
    160.0, 165.0, 170.0, 175.0, 180.0 };

  static const real_T b_coefs[288] = { -0.00071703703703703654,
    -0.00070861598440546025, -0.00015157894736842026, 0.000255999999999998,
    1.3227266504323153E-18, 7.20000000000004E-5, 4.9739130434782169E-5,
    0.00023373913043478285, -0.00022905263157894747, 6.561403508771935E-5,
    0.00022231884057971008, -0.00017707509881422778, 8.72727272727209E-5,
    -7.3846153846147123E-5, 0.00012763532763532021, 4.8148148148149524E-5,
    -0.00023629629629629608, 0.00066370370370370281, -0.00034133333333333308,
    0.0006720000000000004, -0.00029714285714285709, 0.00043174603174603169,
    -0.00014222222222222191, 0.00035555555555555416, -0.000133333333333333,
    6.9388939039072093E-20, -0.00017777777777777849, 0.0012248888888888917,
    4.2666666666666676E-5, -0.0010000000000000013, -0.00016799999999999926,
    -0.00021147826086956564, -0.00020137299771167045, 0.00024089314194577385,
    -0.00012621212121212145, 7.5000000000000061E-5, 7.5000000000000061E-5,
    -0.00012621212121212145, 0.00024089314194577385, -0.00020137299771167045,
    -0.00021147826086956564, -0.00016799999999999926, -0.0010000000000000013,
    4.2666666666666676E-5, 0.0012248888888888917, -0.00017777777777777849,
    6.9388939039072093E-20, -0.000133333333333333, 0.00035555555555555416,
    -0.00014222222222222191, 0.00043174603174603169, -0.00029714285714285709,
    0.0006720000000000004, -0.00034133333333333308, 0.00066370370370370281,
    -0.00023629629629629608, 4.8148148148149524E-5, 0.00012763532763532021,
    -7.3846153846147123E-5, 8.72727272727209E-5, -0.00017707509881422778,
    0.00022231884057971008, 6.561403508771935E-5, -0.00022905263157894747,
    0.00023373913043478285, 4.9739130434782169E-5, 7.20000000000004E-5,
    1.3227266504323153E-18, 0.000255999999999998, -0.00015157894736842026,
    -0.00070861598440546025, -0.00071703703703703654, -0.0052148148148148145,
    -0.0016717348927875152, -0.00088421052631579072, -0.0028799999999999824,
    -0.00048000000000001378, -0.0010800000000000024, -0.00068869565217390886,
    -0.0013773913043478275, 0.0017852631578947377, -0.0010228070175438606,
    -0.001544927536231884, 0.0014071146245059208, -0.0008727272727272323,
    0.00036923076923071343, -0.0010689458689458079, -4.8148148148161735E-5,
    0.001548148148148147, -0.0045037037037036981, 0.0027733333333333316,
    -0.0043200000000000009, 0.002285714285714285, -0.0038730158730158754,
    0.0010666666666666645, -0.0022222222222222127, 0.0013333333333333307,
    2.4286128663675263E-18, 0.00088888888888889565, -0.0084355555555555815,
    0.00037333333333333392, 0.011000000000000013, 0.002239999999999992,
    0.0016973913043478298, 0.00062425629290617794, -0.0022149920255183433,
    0.000837121212121214, -0.00075000000000000067, -0.00037500000000000033,
    0.0010560606060606075, -0.0013984051036682642, 0.0023963386727688788,
    0.0014747826086956544, 0.00027999999999999683, 0.0040000000000000088,
    -0.001013333333333334, -0.0099377777777777954, 0.0017777777777777818,
    -3.4694469519536092E-18, 0.00066666666666666122, -0.0031111111111111,
    0.0010666666666666645, -0.0026031746031746003, 0.0021714285714285711,
    -0.0057600000000000056, 0.0023466666666666644, -0.0054518518518518432,
    0.0019962962962962946, -0.00067407407407408105, -0.00084558404558400079,
    0.00073846153846148714, -0.00043636363636358492, 0.0012490118577074957,
    -0.0017898550724637675, 3.859649122807049E-5, 0.0016505263157894747,
    -0.0021286956521739155, -5.73913043478237E-5, -3.4694469519536142E-18,
    0.00047999999999999394, -0.00095999999999998734, 0.0031578947368420948,
    0.012300974658869418, 0.015970370370370364, 0.19599999999999998,
    0.0900740740740741, 0.020210526315789425, 0.0, -0.0095999999999999749,
    -0.014400000000000013, -0.019800000000000009, -0.022956521739130435,
    -0.019199999999999995, -0.018526315789473679, -0.023833333333333335,
    -0.022608695652173917, -0.021818181818181792, -0.024000000000000035,
    -0.025846153846153894, -0.026962962962962932, -0.023833333333333335,
    -0.026074074074074072, -0.021333333333333346, -0.019200000000000009,
    -0.011999999999999983, -0.011428571428571413, -0.017777777777777788,
    -0.017777777777777788, -0.013333333333333352, -0.010000000000000009,
    -0.00999999999999998, -0.014444444444444411, -0.00693333333333334, 0.0,
    0.035000000000000017, 0.044799999999999993, 0.045913043478260869,
    0.037052631578947365, 0.032969696969696968, 0.031875, 0.03, 0.031875,
    0.032969696969696968, 0.037052631578947365, 0.045913043478260869,
    0.044799999999999993, 0.035000000000000017, 0.0, -0.00693333333333334,
    -0.014444444444444411, -0.00999999999999998, -0.010000000000000009,
    -0.013333333333333352, -0.017777777777777788, -0.017777777777777788,
    -0.011428571428571413, -0.011999999999999983, -0.019200000000000009,
    -0.021333333333333346, -0.026074074074074072, -0.023833333333333335,
    -0.026962962962962932, -0.025846153846153894, -0.024000000000000035,
    -0.021818181818181792, -0.022608695652173917, -0.023833333333333335,
    -0.018526315789473679, -0.019199999999999995, -0.022956521739130435,
    -0.019800000000000009, -0.014400000000000013, -0.0095999999999999749, 0.0,
    0.020210526315789425, 0.0900740740740741, -0.0, 0.76, 1.08, 1.14, 1.1, 1.04,
    0.95, 0.84, 0.72, 0.64, 0.53, 0.4, 0.3, 0.18, 0.06, -0.08, -0.21, -0.32,
    -0.48, -0.56, -0.68, -0.72, -0.82, -0.9, -1.0, -1.05, -1.1, -1.15, -1.28,
    -1.3, -1.15, -0.94, -0.7, -0.48, -0.32, -0.15, -0.0, 0.15, 0.32, 0.48, 0.7,
    0.94, 1.15, 1.3, 1.28, 1.15, 1.1, 1.05, 1.0, 0.9, 0.82, 0.72, 0.68, 0.56,
    0.48, 0.32, 0.21, 0.08, -0.06, -0.18, -0.3, -0.4, -0.53, -0.64, -0.72, -0.84,
    -0.95, -1.04, -1.1, -1.14, -1.08, -0.76 };

  static const real_T c_coefs[288] = { -2.9629629629629451E-6,
    -0.00041820105820105784, -0.00012190476190476223, 0.00042666666666666688,
    -0.0010084848484848488, 0.00033818181818181777, -0.00046666666666666439,
    -0.00013333333333333377, 8.0000000000000074E-5, -2.1333333333333328E-5,
    -9.6000000000000043E-5, 8.5333333333333176E-5, -0.00042666666666666303,
    5.3333333333334634E-5, -2.6666666666667232E-5, 0.00048000000000000093,
    -8.0000000000000074E-5, -5.3333333333333387E-5, 0.00010666666666666676,
    -0.00010666666666666637, 0.00042666666666666428, -0.00017066666666666646,
    2.7428571428571726E-5, -3.6571428571429484E-5, 0.00038400000000000321,
    -0.00012800000000000037, 0.0, -2.7755575615628914E-19,
    -0.0001010526315789473, 0.00022694736842105307, 1.799999999999955E-5,
    0.00015400000000000011, -4.7999999999999988E-5, -3.2000000000000114E-5,
    2.0816681711721684E-19, 0.00015999999999999982, -0.00015999999999999982,
    -2.0816681711721684E-19, 3.2000000000000114E-5, 4.7999999999999988E-5,
    -0.00015400000000000011, -1.799999999999955E-5, -0.00022694736842105307,
    0.0001010526315789473, 2.7755575615628914E-19, 0.0, 0.00012800000000000037,
    -0.00038400000000000321, 3.6571428571429484E-5, -2.7428571428571726E-5,
    0.00017066666666666646, -0.00042666666666666428, 0.00010666666666666637,
    -0.00010666666666666676, 5.3333333333333387E-5, 8.0000000000000074E-5,
    -0.00048000000000000093, 2.6666666666667232E-5, -5.3333333333334634E-5,
    0.00042666666666666303, -8.5333333333333176E-5, 9.6000000000000043E-5,
    2.1333333333333328E-5, -8.0000000000000074E-5, 0.00013333333333333377,
    0.00046666666666666439, -0.00033818181818181777, 0.0010084848484848488,
    -0.00042666666666666688, 0.00012190476190476223, 0.00041820105820105784,
    2.9629629629629451E-6, -0.00038518518518518562, 0.0017058201058201034,
    -0.0009142857142857134, -0.0032000000000000015, 0.00717575757575758,
    -0.0027818181818181796, 0.0033333333333333166, 3.4694469519536142E-18,
    -0.0006666666666666674, 0.00037333333333333354, 0.00072000000000000037,
    -0.00058666666666666611, 0.0026666666666666505, -0.00066666666666667662,
    -0.00013333333333332613, -0.0036000000000000077, 0.00080000000000000069,
    0.00026666666666666695, -0.0010666666666666676, 0.00053333333333333184,
    -0.0031999999999999819, 0.0013866666666666641, -0.00061714285714285885,
    -0.00027428571428570749, -0.0028800000000000197, 0.0012800000000000049, 0.0,
    1.3877787807814458E-18, 0.00050526315789473574, -0.0018294736842105292,
    0.00027000000000000353, -0.00042000000000000088, 0.00072000000000000015,
    0.00032000000000000068, -1.5612511283791262E-18, -0.00079999999999999863,
    0.001599999999999999, 1.5612511283791262E-18, -0.0001600000000000011,
    3.2526065174565133E-19, 0.0018900000000000006, 0.00053999999999999675,
    0.0015747368421052668, -0.0010105263157894737, -2.7755575615628915E-18, 0.0,
    -0.00064000000000000352, 0.0028800000000000197, -0.00082285714285714969,
    -0.00020571428571428298, -0.0011733333333333329, 0.0031999999999999819,
    -0.0010666666666666637, 0.00053333333333333379, -0.0005333333333333339,
    -0.00040000000000000034, 0.0036000000000000051, -0.00053333333333333574,
    0.00013333333333333849, -0.0037333333333333138, 0.00069333333333333139,
    -0.00072000000000000037, 5.3333333333333618E-5, 0.0005333333333333339,
    -0.0020000000000000035, -0.0036666666666666497, 0.0022909090909090865,
    -0.0079515151515151521, 0.0032000000000000015, -0.0027428571428571467,
    -0.004567195767195764, -0.00042962962962962979, 0.05800000000000001,
    0.053925925925925933, 0.03961904761904763, 0.021333333333333329,
    0.021333333333333329, 0.017454545454545466, 0.015, 0.013333333333333336,
    0.0033333333333333366, 0.0026666666666666692, 0.0048000000000000048,
    0.0048000000000000048, 0.0053333333333333314, 0.0, -0.0026666666666666753,
    -0.0059999999999999941, -0.0060000000000000053, -0.0040000000000000036,
    -0.0053333333333333384, -0.0080000000000000071, -0.010666666666666666,
    -0.010666666666666666, -0.00960000000000001, -0.013714285714285719,
    -0.019200000000000005, -0.019200000000000005, -0.015999999999999993,
    -0.015999999999999993, -0.016, -0.018526315789473689, -0.0198, -0.01575,
    -0.0084000000000000012, -0.0047999999999999987, -0.004000000000000001,
    -0.004000000000000001, 0.0, 0.004000000000000001, 0.004000000000000001,
    0.0047999999999999987, 0.0084000000000000012, 0.01575, 0.0198,
    0.018526315789473689, 0.016, 0.015999999999999993, 0.015999999999999993,
    0.019200000000000005, 0.019200000000000005, 0.013714285714285719,
    0.00960000000000001, 0.010666666666666666, 0.010666666666666666,
    0.0080000000000000071, 0.0053333333333333384, 0.0040000000000000036,
    0.0060000000000000053, 0.0059999999999999941, 0.0026666666666666753, 0.0,
    -0.0053333333333333314, -0.0048000000000000048, -0.0048000000000000048,
    -0.0026666666666666692, -0.0033333333333333366, -0.013333333333333336,
    -0.015, -0.017454545454545466, -0.021333333333333329, -0.021333333333333329,
    -0.03961904761904763, -0.053925925925925933, 0.1, 0.38, 0.64, 0.8, 0.88,
    1.04, 1.1, 1.2, 1.25, 1.26, 1.28, 1.31, 1.33, 1.37, 1.36, 1.34, 1.28, 1.26,
    1.24, 1.2, 1.16, 1.08, 1.04, 0.98, 0.9, 0.78, 0.7, 0.62, 0.54, 0.46, 0.35,
    0.26, 0.19, 0.16, 0.14, 0.12, 0.1, 0.12, 0.14, 0.16, 0.19, 0.26, 0.35, 0.46,
    0.54, 0.62, 0.7, 0.78, 0.9, 0.98, 1.04, 1.08, 1.16, 1.2, 1.24, 1.26, 1.28,
    1.34, 1.36, 1.37, 1.33, 1.31, 1.28, 1.26, 1.25, 1.2, 1.1, 1.04, 0.88, 0.8,
    0.64, 0.38 };

  for (k = 0; k < 101; k++) {
    attack_0 = attack[k] / 3.1415926535897931 * 180.0;
    if (rtIsNaN(attack_0)) {
      Cls[k] = (rtNaN);
      Cds[k] = (rtNaN);
    } else {
      Cls[k] = boat_controller_ppval(b_breaks, b_coefs, attack_0);
      Cds[k] = boat_controller_ppval(b_breaks, c_coefs, attack_0);
    }

    attack[k] = attack_0;
  }
}

static void matlabCodegenHandle_matlabCod_m(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void boat_controller_step(void)
{
  boolean_T rEQ0;
  int32_T idx;
  int32_T k;
  int64_T tmp;
  real_T varargin_1_tmp;
  static const real_T b[101] = { 0.0, 0.031415926535897934, 0.062831853071795868,
    0.0942477796076938, 0.12566370614359174, 0.15707963267948966,
    0.1884955592153876, 0.21991148575128555, 0.25132741228718347,
    0.28274333882308139, 0.31415926535897931, 0.34557519189487729,
    0.37699111843077521, 0.40840704496667313, 0.4398229715025711,
    0.471238898038469, 0.50265482457436694, 0.53407075111026492,
    0.56548667764616278, 0.59690260418206076, 0.62831853071795862,
    0.6597344572538566, 0.69115038378975457, 0.72256631032565244,
    0.75398223686155041, 0.78539816339744839, 0.81681408993334625,
    0.84823001646924423, 0.87964594300514221, 0.91106186954104007,
    0.942477796076938, 0.97389372261283591, 1.0053096491487339,
    1.0367255756846319, 1.0681415022205298, 1.0995574287564276,
    1.1309733552923256, 1.1623892818282235, 1.1938052083641215,
    1.2252211349000195, 1.2566370614359172, 1.2880529879718152,
    1.3194689145077132, 1.3508848410436112, 1.3823007675795091,
    1.4137166941154071, 1.4451326206513049, 1.4765485471872029,
    1.5079644737231008, 1.5393804002589988, 1.5707963267948966,
    1.6022122533307943, 1.6336281798666923, 1.6650441064025903,
    1.6964600329384882, 1.727875959474386, 1.759291886010284, 1.7907078125461819,
    1.82212373908208, 1.8535396656179779, 1.8849555921538759, 1.9163715186897736,
    1.9477874452256716, 1.9792033717615696, 2.0106192982974678,
    2.0420352248333655, 2.0734511513692633, 2.104867077905161,
    2.1362830044410592, 2.1676989309769574, 2.1991148575128552,
    2.2305307840487529, 2.2619467105846507, 2.2933626371205489,
    2.3247785636564471, 2.3561944901923448, 2.3876104167282426,
    2.4190263432641408, 2.4504422698000385, 2.4818581963359367,
    2.5132741228718345, 2.5446900494077322, 2.5761059759436304,
    2.6075219024795282, 2.6389378290154264, 2.6703537555513241,
    2.7017696820872219, 2.73318560862312, 2.7646015351590179, 2.7960174616949161,
    2.8274333882308138, 2.8588493147667116, 2.8902652413026098,
    2.9216811678385075, 2.9530970943744057, 2.9845130209103035,
    3.0159289474462012, 3.0473448739820994, 3.0787608005179972,
    3.1101767270538954, 3.1415926535897931 };

  boolean_T exitg1;

  // Reset subsysRan breadcrumbs
  srClearBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC_i);

  // Reset subsysRan breadcrumbs
  srClearBC(boat_controller_DW.EnabledSubsystem1_SubsysRanBC);

  // Reset subsysRan breadcrumbs
  srClearBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC_k);

  // Reset subsysRan breadcrumbs
  srClearBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S6>/SourceBlock'
  rEQ0 = Sub_boat_controller_192.getLatestMessage
    (&boat_controller_B.b_varargout_2_m);

  // Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem1' incorporates:
  //   EnablePort: '<S3>/Enable'

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (rEQ0) {
    srUpdateBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC);

    // Gain: '<S10>/Gain1'
    boat_controller_B.Gain1 = boat_controller_P.Gain1_Gain *
      boat_controller_B.b_varargout_2_m.HeadingDegrees;
    srUpdateBC(boat_controller_DW.EnabledSubsystem1_SubsysRanBC);
  }

  // End of MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Enabled Subsystem1'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S5>/SourceBlock'
  rEQ0 = Sub_boat_controller_97.getLatestMessage
    (&boat_controller_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S2>/Enable'

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S11>/Enable'

  if (rEQ0) {
    srUpdateBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC_k);

    // Gain: '<S9>/Gain1'
    tmp = static_cast<int64_T>(static_cast<int64_T>
      (boat_controller_P.Gain1_Gain_l) * static_cast<int64_T>
      (boat_controller_B.b_varargout_2.Gps0TrueHeading));
    if (tmp > 2147483647L) {
      tmp = 2147483647L;
    } else {
      if (tmp < -2147483648L) {
        tmp = -2147483648L;
      }
    }

    // DataTypeConversion: '<S2>/Cast To Double1' incorporates:
    //   Gain: '<S9>/Gain1'

    boat_controller_B.CastToDouble1 = static_cast<real_T>(static_cast<int32_T>
      (tmp));

    // Gain: '<S8>/Gain1'
    tmp = static_cast<int64_T>(static_cast<int64_T>
      (boat_controller_P.Gain1_Gain_n) * static_cast<int64_T>
      (boat_controller_B.b_varargout_2.WindSensor0Direction));
    if (tmp > 2147483647L) {
      tmp = 2147483647L;
    } else {
      if (tmp < -2147483648L) {
        tmp = -2147483648L;
      }
    }

    // DataTypeConversion: '<S2>/Cast To Double2' incorporates:
    //   Gain: '<S8>/Gain1'

    boat_controller_B.CastToDouble2 = static_cast<real_T>(static_cast<int32_T>
      (tmp));
    srUpdateBC(boat_controller_DW.EnabledSubsystem_SubsysRanBC_i);
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLAB Function: '<S7>/MATLAB Function'
  boat_controller_B.error = boat_controller_B.Gain1 -
    boat_controller_B.CastToDouble1;
  if (fabs(boat_controller_B.error) > 3.1415926535897931) {
    if (rtIsNaN(boat_controller_B.error + 3.1415926535897931)) {
      boat_controller_B.sailAngle = (rtNaN);
    } else if (rtIsInf(boat_controller_B.error + 3.1415926535897931)) {
      boat_controller_B.sailAngle = (rtNaN);
    } else if (boat_controller_B.error + 3.1415926535897931 == 0.0) {
      boat_controller_B.sailAngle = 0.0;
    } else {
      boat_controller_B.sailAngle = fmod(boat_controller_B.error +
        3.1415926535897931, 6.2831853071795862);
      rEQ0 = (boat_controller_B.sailAngle == 0.0);
      if (!rEQ0) {
        boat_controller_B.q = fabs((boat_controller_B.error + 3.1415926535897931)
          / 6.2831853071795862);
        rEQ0 = !(fabs(boat_controller_B.q - floor(boat_controller_B.q + 0.5)) >
                 2.2204460492503131E-16 * boat_controller_B.q);
      }

      if (rEQ0) {
        boat_controller_B.sailAngle = 0.0;
      } else {
        if (boat_controller_B.error + 3.1415926535897931 < 0.0) {
          boat_controller_B.sailAngle += 6.2831853071795862;
        }
      }
    }

    if ((boat_controller_B.sailAngle == 0.0) && (boat_controller_B.error +
         3.1415926535897931 > 0.0)) {
      boat_controller_B.sailAngle = 6.2831853071795862;
    }

    boat_controller_B.error = boat_controller_B.sailAngle - 3.1415926535897931;
  }

  // End of MATLAB Function: '<S7>/MATLAB Function'

  // Gain: '<S54>/Proportional Gain'
  boat_controller_B.sailAngle = boat_controller_P.PIDController_P *
    boat_controller_B.error;

  // Saturate: '<S7>/rudder angle saturation'
  if (boat_controller_B.sailAngle >
      boat_controller_P.rudderanglesaturation_UpperSat) {
    boat_controller_B.rudderanglesaturation =
      boat_controller_P.rudderanglesaturation_UpperSat;
  } else if (boat_controller_B.sailAngle <
             boat_controller_P.rudderanglesaturation_LowerSat) {
    boat_controller_B.rudderanglesaturation =
      boat_controller_P.rudderanglesaturation_LowerSat;
  } else {
    boat_controller_B.rudderanglesaturation = boat_controller_B.sailAngle;
  }

  // End of Saturate: '<S7>/rudder angle saturation'

  // MATLAB Function: '<S7>/MATLAB Function2'
  boat_controller_B.q = fabs(boat_controller_B.CastToDouble2);
  memcpy(&boat_controller_B.b[0], &b[0], static_cast<uint32_T>(101 * sizeof
          (real_T)));
  boat_controller_sailcoef(boat_controller_B.b, boat_controller_B.Cls,
    boat_controller_B.Cds);
  boat_controller_B.sailAngle = sin(3.1415926535897931 - boat_controller_B.q);
  boat_controller_B.q = cos(3.1415926535897931 - boat_controller_B.q);
  for (idx = 0; idx < 101; idx++) {
    varargin_1_tmp = boat_controller_B.Cls[idx] * boat_controller_B.sailAngle -
      boat_controller_B.Cds[idx] * boat_controller_B.q;
    boat_controller_B.b[idx] = varargin_1_tmp;
    boat_controller_B.c_x_data[idx] = varargin_1_tmp;
  }

  if (!rtIsNaN(boat_controller_B.c_x_data[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 102)) {
      if (!rtIsNaN(boat_controller_B.c_x_data[static_cast<int32_T>(k - 1)])) {
        idx = k;
        exitg1 = true;
      } else {
        k = static_cast<int32_T>(k + 1);
      }
    }
  }

  if (idx == 0) {
    boat_controller_B.sailAngle = boat_controller_B.b[0];
  } else {
    boat_controller_B.sailAngle = boat_controller_B.b[static_cast<int32_T>(idx -
      1)];
    while (static_cast<int32_T>(idx + 1) < 102) {
      if (boat_controller_B.sailAngle < boat_controller_B.b[idx]) {
        boat_controller_B.sailAngle = boat_controller_B.b[idx];
      }

      idx = static_cast<int32_T>(idx + 1);
    }
  }

  // End of MATLAB Function: '<S7>/MATLAB Function2'

  // Saturate: '<Root>/Saturation'
  if (boat_controller_B.rudderanglesaturation >
      boat_controller_P.Saturation_UpperSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Rudder =
      boat_controller_P.Saturation_UpperSat;
  } else if (boat_controller_B.rudderanglesaturation <
             boat_controller_P.Saturation_LowerSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Rudder =
      boat_controller_P.Saturation_LowerSat;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Rudder =
      boat_controller_B.rudderanglesaturation;
  }

  // End of Saturate: '<Root>/Saturation'

  // Saturate: '<Root>/Saturation1'
  if (boat_controller_B.sailAngle > boat_controller_P.Saturation1_UpperSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Winch =
      boat_controller_P.Saturation1_UpperSat;
  } else if (boat_controller_B.sailAngle <
             boat_controller_P.Saturation1_LowerSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Winch =
      boat_controller_P.Saturation1_LowerSat;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    boat_controller_B.BusAssignment.Winch = boat_controller_B.sailAngle;
  }

  // End of Saturate: '<Root>/Saturation1'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S4>/SinkBlock'
  Pub_boat_controller_103.publish(&boat_controller_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // External mode
  rtExtModeUploadCheckTrigger(2);

  {                                    // Sample time: [0.0s, 0.0s]
    rtExtModeUpload(0, (real_T)boat_controller_M->Timing.t[0]);
  }

  {                                    // Sample time: [0.2s, 0.0s]
    rtExtModeUpload(1, (real_T)((boat_controller_M->Timing.clockTick1) * 0.2));
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.0s, 0.0s]
    if ((rtmGetTFinal(boat_controller_M)!=-1) &&
        !((rtmGetTFinal(boat_controller_M)-boat_controller_M->Timing.t[0]) >
          boat_controller_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(boat_controller_M, "Simulation finished");
    }

    if (rtmGetStopRequested(boat_controller_M)) {
      rtmSetErrorStatus(boat_controller_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  boat_controller_M->Timing.t[0] =
    ((time_T)(++boat_controller_M->Timing.clockTick0)) *
    boat_controller_M->Timing.stepSize0;

  {
    // Update absolute timer for sample time: [0.2s, 0.0s]
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.2, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    boat_controller_M->Timing.clockTick1++;
  }
}

// Model initialize function
void boat_controller_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&boat_controller_M->solverInfo,
                          &boat_controller_M->Timing.simTimeStep);
    rtsiSetTPtr(&boat_controller_M->solverInfo, &rtmGetTPtr(boat_controller_M));
    rtsiSetStepSizePtr(&boat_controller_M->solverInfo,
                       &boat_controller_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&boat_controller_M->solverInfo, (&rtmGetErrorStatus
      (boat_controller_M)));
    rtsiSetRTModelPtr(&boat_controller_M->solverInfo, boat_controller_M);
  }

  rtsiSetSimTimeStep(&boat_controller_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&boat_controller_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(boat_controller_M, &boat_controller_M->Timing.tArray[0]);
  rtmSetTFinal(boat_controller_M, -1);
  boat_controller_M->Timing.stepSize0 = 0.2;

  // External mode info
  boat_controller_M->Sizes.checksums[0] = (3852560315U);
  boat_controller_M->Sizes.checksums[1] = (2405753417U);
  boat_controller_M->Sizes.checksums[2] = (1926533184U);
  boat_controller_M->Sizes.checksums[3] = (3900680471U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[13];
    boat_controller_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = (sysRanDType *)
      &boat_controller_DW.EnabledSubsystem_SubsysRanBC_i;
    systemRan[2] = (sysRanDType *)
      &boat_controller_DW.EnabledSubsystem1_SubsysRanBC;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = (sysRanDType *)
      &boat_controller_DW.EnabledSubsystem_SubsysRanBC_k;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = (sysRanDType *)
      &boat_controller_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[10] = &rtAlwaysEnabled;
    systemRan[11] = &rtAlwaysEnabled;
    systemRan[12] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(boat_controller_M->extModeInfo,
      &boat_controller_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(boat_controller_M->extModeInfo,
                        boat_controller_M->Sizes.checksums);
    rteiSetTPtr(boat_controller_M->extModeInfo, rtmGetTPtr(boat_controller_M));
  }

  // block I/O
  (void) memset((static_cast<void *>(&boat_controller_B)), 0,
                sizeof(B_boat_controller_T));

  // states (dwork)
  (void) memset(static_cast<void *>(&boat_controller_DW), 0,
                sizeof(DW_boat_controller_T));

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    boat_controller_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 24;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    char_T tmp[17];
    char_T tmp_0[9];
    char_T tmp_1[30];
    int32_T i;
    static const char_T tmp_2[16] = { '/', 'h', 'e', 'a', 'd', 'i', 'n', 'g',
      '_', 'd', 'e', 'g', 'r', 'e', 'e', 's' };

    static const char_T tmp_3[8] = { '/', 's', 'e', 'n', 's', 'o', 'r', 's' };

    static const char_T tmp_4[29] = { '/', 'r', 'u', 'd', 'd', 'e', 'r', '_',
      'w', 'i', 'n', 'c', 'h', '_', 'a', 'c', 't', 'u', 'a', 't', 'i', 'o', 'n',
      '_', 'a', 'n', 'g', 'l', 'e' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S6>/SourceBlock'
    boat_controller_DW.obj_p.matlabCodegenIsDeleted = false;
    boat_controller_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[16] = '\x00';
    Sub_boat_controller_192.createSubscriber(tmp, 1);
    boat_controller_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Enabled SubSystem: '<Root>/Enabled Subsystem1'
    // SystemInitialize for Outport: '<S3>/desiredHeadingRad'
    boat_controller_B.Gain1 = boat_controller_P.desiredHeadingRad_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/Enabled Subsystem1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S5>/SourceBlock'
    boat_controller_DW.obj_e.matlabCodegenIsDeleted = false;
    boat_controller_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_0[i] = tmp_3[i];
    }

    tmp_0[8] = '\x00';
    Sub_boat_controller_97.createSubscriber(tmp_0, 1);
    boat_controller_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Enabled SubSystem: '<Root>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S2>/Gps0TrueHeadingRad'
    boat_controller_B.CastToDouble1 = boat_controller_P.Gps0TrueHeadingRad_Y0;

    // SystemInitialize for Outport: '<S2>/WindSensor0DirectionRad'
    boat_controller_B.CastToDouble2 =
      boat_controller_P.WindSensor0DirectionRad_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/Enabled Subsystem'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    boat_controller_DW.obj.matlabCodegenIsDeleted = false;
    boat_controller_DW.obj.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      tmp_1[i] = tmp_4[i];
    }

    tmp_1[29] = '\x00';
    Pub_boat_controller_103.createPublisher(tmp_1, 1);
    boat_controller_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'
  }
}

// Model terminate function
void boat_controller_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  matlabCodegenHandle_matlabCod_m(&boat_controller_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCod_m(&boat_controller_DW.obj_e);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&boat_controller_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
