/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Controller_Pioneer3DXAssembly_sfun.h"
#include "c2_Controller_Pioneer3DXAssembly.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Controller_Pioneer3DXAssembly_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[20] = { "r_com", "r_range_m", "r_b",
  "yb_b", "r_f", "yb_f", "SensorPos", "SensorPosNew", "i", "theta", "rotZ90",
  "rotMatNew", "nargin", "nargout", "x", "y", "rotMat", "r_range", "xRange",
  "yRange" };

/* Function Declarations */
static void initialize_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void initialize_params_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void enable_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void disable_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void set_sim_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_st);
static void finalize_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void sf_gateway_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void c2_chartstep_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void initSimStructsc2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, const mxArray *c2_yRange, const char_T *c2_identifier, real_T
  c2_y[16]);
static void c2_b_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[16]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[8]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_d_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_e_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[48]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[32]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u);
static real_T c2_sind(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T c2_x);
static boolean_T c2_isfinite(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, real_T c2_x);
static real_T c2_cosd(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T c2_x);
static void c2_eml_scalar_eg(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static void c2_threshold(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *
  chartInstance);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_h_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_i_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_Controller_Pioneer3DXAssembly, const char_T
   *c2_identifier);
static uint8_T c2_j_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sind(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T *c2_x);
static void c2_b_cosd(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T *c2_x);
static void init_dsm_address_info
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_Controller_Pioneer3DXAssembly = 0U;
}

static void initialize_params_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  real_T c2_dv0[8];
  int32_T c2_i0;
  sf_mex_import_named("r_range", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      c2_dv0, 0, 0, 0U, 1, 0U, 1, 8);
  for (c2_i0 = 0; c2_i0 < 8; c2_i0++) {
    chartInstance->c2_r_range[c2_i0] = c2_dv0[c2_i0];
  }
}

static void enable_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i1;
  real_T c2_u[16];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i2;
  real_T c2_b_u[16];
  const mxArray *c2_c_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T (*c2_yRange)[16];
  real_T (*c2_xRange)[16];
  c2_yRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_xRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(3, 1), false);
  for (c2_i1 = 0; c2_i1 < 16; c2_i1++) {
    c2_u[c2_i1] = (*c2_xRange)[c2_i1];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 16), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i2 = 0; c2_i2 < 16; c2_i2++) {
    c2_b_u[c2_i2] = (*c2_yRange)[c2_i2];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 1, 16), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_hoistedGlobal =
    chartInstance->c2_is_active_c2_Controller_Pioneer3DXAssembly;
  c2_c_u = c2_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv1[16];
  int32_T c2_i3;
  real_T c2_dv2[16];
  int32_T c2_i4;
  real_T (*c2_xRange)[16];
  real_T (*c2_yRange)[16];
  c2_yRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_xRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                      "xRange", c2_dv1);
  for (c2_i3 = 0; c2_i3 < 16; c2_i3++) {
    (*c2_xRange)[c2_i3] = c2_dv1[c2_i3];
  }

  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                      "yRange", c2_dv2);
  for (c2_i4 = 0; c2_i4 < 16; c2_i4++) {
    (*c2_yRange)[c2_i4] = c2_dv2[c2_i4];
  }

  chartInstance->c2_is_active_c2_Controller_Pioneer3DXAssembly =
    c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
    "is_active_c2_Controller_Pioneer3DXAssembly");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Controller_Pioneer3DXAssembly(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  real_T *c2_x;
  real_T *c2_y;
  real_T (*c2_yRange)[16];
  real_T (*c2_xRange)[16];
  real_T (*c2_rotMat)[9];
  c2_yRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_xRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_rotMat = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c2_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_x, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_y, 1U);
  for (c2_i5 = 0; c2_i5 < 9; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_rotMat)[c2_i5], 2U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_Controller_Pioneer3DXAssembly(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_Controller_Pioneer3DXAssemblyMachineNumber_, chartInstance->chartNumber,
     chartInstance->instanceNumber);
  for (c2_i6 = 0; c2_i6 < 16; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*c2_xRange)[c2_i6], 3U);
  }

  for (c2_i7 = 0; c2_i7 < 16; c2_i7++) {
    _SFD_DATA_RANGE_CHECK((*c2_yRange)[c2_i7], 4U);
  }

  for (c2_i8 = 0; c2_i8 < 8; c2_i8++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c2_r_range[c2_i8], 5U);
  }
}

static void c2_chartstep_c2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_x;
  real_T c2_y;
  int32_T c2_i9;
  real_T c2_rotMat[9];
  int32_T c2_i10;
  real_T c2_b_r_range[8];
  uint32_T c2_debug_family_var_map[20];
  real_T c2_r_com;
  real_T c2_r_range_m[8];
  real_T c2_r_b[8];
  real_T c2_yb_b;
  real_T c2_r_f[8];
  real_T c2_yb_f;
  real_T c2_SensorPos[32];
  real_T c2_SensorPosNew[48];
  real_T c2_i;
  real_T c2_theta;
  real_T c2_rotZ90[9];
  real_T c2_rotMatNew[9];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 2.0;
  real_T c2_xRange[16];
  real_T c2_yRange[16];
  int32_T c2_i11;
  real_T c2_A[8];
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  int32_T c2_b_theta;
  real_T c2_d0;
  real_T c2_d1;
  int32_T c2_b_i;
  int32_T c2_c_theta;
  real_T c2_d2;
  real_T c2_d3;
  int32_T c2_c_i;
  int32_T c2_i17;
  static real_T c2_b[9] = { 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c2_i18;
  real_T c2_a[9];
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  real_T c2_C[9];
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_d_i;
  int32_T c2_i30;
  int32_T c2_e_i;
  int32_T c2_i31;
  real_T c2_b_b[3];
  int32_T c2_i32;
  real_T c2_b_y[3];
  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_f_i;
  real_T c2_b_x[3];
  int32_T c2_i35;
  int32_T c2_i36;
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  real_T *c2_c_x;
  real_T *c2_c_y;
  real_T (*c2_b_xRange)[16];
  real_T (*c2_b_yRange)[16];
  real_T (*c2_b_rotMat)[9];
  c2_b_yRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_xRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_rotMat = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c2_c_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_c_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_c_x;
  c2_b_hoistedGlobal = *c2_c_y;
  c2_x = c2_hoistedGlobal;
  c2_y = c2_b_hoistedGlobal;
  for (c2_i9 = 0; c2_i9 < 9; c2_i9++) {
    c2_rotMat[c2_i9] = (*c2_b_rotMat)[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 8; c2_i10++) {
    c2_b_r_range[c2_i10] = chartInstance->c2_r_range[c2_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 20U, 20U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r_com, 0U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r_range_m, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r_b, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yb_b, 3U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r_f, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yb_f, 5U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_SensorPos, 6U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_SensorPosNew, 7U, c2_e_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_i, 8U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 9U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_rotZ90, 10U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rotMatNew, 11U, c2_c_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 12U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 13U, c2_d_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_x, 14U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_y, 15U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_rotMat, 16U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_r_range, 17U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_xRange, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_yRange, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_r_com = 0.135;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i11 = 0; c2_i11 < 8; c2_i11++) {
    c2_A[c2_i11] = c2_b_r_range[c2_i11];
  }

  for (c2_i12 = 0; c2_i12 < 8; c2_i12++) {
    c2_r_range_m[c2_i12] = c2_A[c2_i12] / 1000.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  for (c2_i13 = 0; c2_i13 < 8; c2_i13++) {
    c2_r_b[c2_i13] = c2_r_com + c2_r_range_m[c2_i13];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_yb_b = 0.18;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  for (c2_i14 = 0; c2_i14 < 8; c2_i14++) {
    c2_r_f[c2_i14] = c2_r_com + c2_r_range_m[c2_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_yb_f = 0.09;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  for (c2_i15 = 0; c2_i15 < 32; c2_i15++) {
    c2_SensorPos[c2_i15] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  for (c2_i16 = 0; c2_i16 < 48; c2_i16++) {
    c2_SensorPosNew[c2_i16] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_SensorPos[0] = -c2_r_f[0];
  c2_SensorPos[16] = c2_r_f[0] * 0.0 + c2_yb_f;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_SensorPos[7] = c2_r_f[7];
  c2_SensorPos[23] = c2_r_f[7] * -0.0 + c2_yb_f;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  c2_i = 2.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_theta = -50.0;
  c2_b_theta = 0;
  while (c2_b_theta < 6) {
    c2_theta = -50.0 + (real_T)c2_b_theta * 20.0;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
    c2_d0 = c2_theta;
    c2_b_sind(chartInstance, &c2_d0);
    c2_d1 = c2_theta;
    c2_b_cosd(chartInstance, &c2_d1);
    c2_b_i = _SFD_EML_ARRAY_BOUNDS_CHECK("SensorPos", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_i), 1, 16, 1, 0) - 1;
    c2_SensorPos[c2_b_i] = c2_r_f[_SFD_EML_ARRAY_BOUNDS_CHECK("r_f", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_i), 1, 8, 1, 0) - 1] * c2_d0;
    c2_SensorPos[16 + c2_b_i] = c2_r_f[_SFD_EML_ARRAY_BOUNDS_CHECK("r_f",
      (int32_T)_SFD_INTEGER_CHECK("i", c2_i), 1, 8, 1, 0) - 1] * c2_d1 + c2_yb_f;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
    c2_i++;
    c2_b_theta++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  c2_SensorPos[8] = c2_r_b[0];
  c2_SensorPos[24] = c2_r_b[0] * -0.0 - c2_yb_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_SensorPos[15] = -c2_r_b[7];
  c2_SensorPos[31] = c2_r_b[7] * 0.0 - c2_yb_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_i = 10.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_theta = -50.0;
  c2_c_theta = 0;
  while (c2_c_theta < 6) {
    c2_theta = -50.0 + (real_T)c2_c_theta * 20.0;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
    c2_d2 = c2_theta;
    c2_b_sind(chartInstance, &c2_d2);
    c2_d3 = c2_theta;
    c2_b_cosd(chartInstance, &c2_d3);
    c2_c_i = _SFD_EML_ARRAY_BOUNDS_CHECK("SensorPos", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_i), 1, 16, 1, 0) - 1;
    c2_SensorPos[c2_c_i] = -c2_r_b[_SFD_EML_ARRAY_BOUNDS_CHECK("r_b", (int32_T)
      _SFD_INTEGER_CHECK("i-8", c2_i - 8.0), 1, 8, 1, 0) - 1] * c2_d2;
    c2_SensorPos[16 + c2_c_i] = -c2_r_b[_SFD_EML_ARRAY_BOUNDS_CHECK("r_b",
      (int32_T)_SFD_INTEGER_CHECK("i-8", c2_i - 8.0), 1, 8, 1, 0) - 1] * c2_d3 -
      c2_yb_b;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
    c2_i++;
    c2_c_theta++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
  for (c2_i17 = 0; c2_i17 < 9; c2_i17++) {
    c2_rotZ90[c2_i17] = c2_b[c2_i17];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  for (c2_i18 = 0; c2_i18 < 9; c2_i18++) {
    c2_a[c2_i18] = c2_rotMat[c2_i18];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i19 = 0; c2_i19 < 9; c2_i19++) {
    c2_rotMatNew[c2_i19] = 0.0;
  }

  for (c2_i20 = 0; c2_i20 < 9; c2_i20++) {
    c2_rotMatNew[c2_i20] = 0.0;
  }

  for (c2_i21 = 0; c2_i21 < 9; c2_i21++) {
    c2_C[c2_i21] = c2_rotMatNew[c2_i21];
  }

  for (c2_i22 = 0; c2_i22 < 9; c2_i22++) {
    c2_rotMatNew[c2_i22] = c2_C[c2_i22];
  }

  c2_threshold(chartInstance);
  for (c2_i23 = 0; c2_i23 < 9; c2_i23++) {
    c2_C[c2_i23] = c2_rotMatNew[c2_i23];
  }

  for (c2_i24 = 0; c2_i24 < 9; c2_i24++) {
    c2_rotMatNew[c2_i24] = c2_C[c2_i24];
  }

  for (c2_i25 = 0; c2_i25 < 3; c2_i25++) {
    c2_i26 = 0;
    for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
      c2_rotMatNew[c2_i26 + c2_i25] = 0.0;
      c2_i28 = 0;
      for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
        c2_rotMatNew[c2_i26 + c2_i25] += c2_a[c2_i28 + c2_i25] * c2_b[c2_i29 +
          c2_i26];
        c2_i28 += 3;
      }

      c2_i26 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  c2_i = 1.0;
  c2_d_i = 0;
  while (c2_d_i < 16) {
    c2_i = 1.0 + (real_T)c2_d_i;
    CV_EML_FOR(0, 1, 2, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
    for (c2_i30 = 0; c2_i30 < 9; c2_i30++) {
      c2_a[c2_i30] = c2_rotMatNew[c2_i30];
    }

    c2_e_i = _SFD_EML_ARRAY_BOUNDS_CHECK("SensorPos", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_i), 1, 16, 1, 0) - 1;
    for (c2_i31 = 0; c2_i31 < 2; c2_i31++) {
      c2_b_b[c2_i31] = c2_SensorPos[c2_e_i + (c2_i31 << 4)];
    }

    c2_b_b[2] = 0.0;
    c2_b_eml_scalar_eg(chartInstance);
    c2_b_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
      c2_b_y[c2_i32] = 0.0;
      c2_i33 = 0;
      for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
        c2_b_y[c2_i32] += c2_a[c2_i33 + c2_i32] * c2_b_b[c2_i34];
        c2_i33 += 3;
      }
    }

    c2_f_i = _SFD_EML_ARRAY_BOUNDS_CHECK("SensorPosNew", (int32_T)
      _SFD_INTEGER_CHECK("i", c2_i), 1, 16, 1, 0) - 1;
    c2_b_x[0] = c2_x;
    c2_b_x[1] = c2_y;
    c2_b_x[2] = 0.0;
    for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
      c2_SensorPosNew[c2_f_i + (c2_i35 << 4)] = c2_b_y[c2_i35] + c2_b_x[c2_i35];
    }

    c2_d_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 2, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
  for (c2_i36 = 0; c2_i36 < 16; c2_i36++) {
    c2_xRange[c2_i36] = c2_SensorPosNew[c2_i36];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
  for (c2_i37 = 0; c2_i37 < 16; c2_i37++) {
    c2_yRange[c2_i37] = c2_SensorPosNew[c2_i37 + 16];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -43);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i38 = 0; c2_i38 < 16; c2_i38++) {
    (*c2_b_xRange)[c2_i38] = c2_xRange[c2_i38];
  }

  for (c2_i39 = 0; c2_i39 < 16; c2_i39++) {
    (*c2_b_yRange)[c2_i39] = c2_yRange[c2_i39];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_Controller_Pioneer3DXAssembly
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i40;
  real_T c2_b_inData[16];
  int32_T c2_i41;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i40 = 0; c2_i40 < 16; c2_i40++) {
    c2_b_inData[c2_i40] = (*(real_T (*)[16])c2_inData)[c2_i40];
  }

  for (c2_i41 = 0; c2_i41 < 16; c2_i41++) {
    c2_u[c2_i41] = c2_b_inData[c2_i41];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 16), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, const mxArray *c2_yRange, const char_T *c2_identifier, real_T
  c2_y[16])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_yRange), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_yRange);
}

static void c2_b_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[16])
{
  real_T c2_dv3[16];
  int32_T c2_i42;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 1, 16);
  for (c2_i42 = 0; c2_i42 < 16; c2_i42++) {
    c2_y[c2_i42] = c2_dv3[c2_i42];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_yRange;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i43;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_yRange = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_yRange), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_yRange);
  for (c2_i43 = 0; c2_i43 < 16; c2_i43++) {
    (*(real_T (*)[16])c2_outData)[c2_i43] = c2_y[c2_i43];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i44;
  real_T c2_b_inData[8];
  int32_T c2_i45;
  real_T c2_u[8];
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i44 = 0; c2_i44 < 8; c2_i44++) {
    c2_b_inData[c2_i44] = (*(real_T (*)[8])c2_inData)[c2_i44];
  }

  for (c2_i45 = 0; c2_i45 < 8; c2_i45++) {
    c2_u[c2_i45] = c2_b_inData[c2_i45];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[8])
{
  real_T c2_dv4[8];
  int32_T c2_i46;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv4, 1, 0, 0U, 1, 0U, 1, 8);
  for (c2_i46 = 0; c2_i46 < 8; c2_i46++) {
    c2_y[c2_i46] = c2_dv4[c2_i46];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_r_range;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[8];
  int32_T c2_i47;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_b_r_range = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_r_range), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_r_range);
  for (c2_i47 = 0; c2_i47 < 8; c2_i47++) {
    (*(real_T (*)[8])c2_outData)[c2_i47] = c2_y[c2_i47];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  real_T c2_b_inData[9];
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i48 = 0;
  for (c2_i49 = 0; c2_i49 < 3; c2_i49++) {
    for (c2_i50 = 0; c2_i50 < 3; c2_i50++) {
      c2_b_inData[c2_i50 + c2_i48] = (*(real_T (*)[9])c2_inData)[c2_i50 + c2_i48];
    }

    c2_i48 += 3;
  }

  c2_i51 = 0;
  for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
    for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
      c2_u[c2_i53 + c2_i51] = c2_b_inData[c2_i53 + c2_i51];
    }

    c2_i51 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_d_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d4;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d4, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d4;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_e_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9])
{
  real_T c2_dv5[9];
  int32_T c2_i54;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv5, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i54 = 0; c2_i54 < 9; c2_i54++) {
    c2_y[c2_i54] = c2_dv5[c2_i54];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_rotMatNew;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i55;
  int32_T c2_i56;
  int32_T c2_i57;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_rotMatNew = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_rotMatNew), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_rotMatNew);
  c2_i55 = 0;
  for (c2_i56 = 0; c2_i56 < 3; c2_i56++) {
    for (c2_i57 = 0; c2_i57 < 3; c2_i57++) {
      (*(real_T (*)[9])c2_outData)[c2_i57 + c2_i55] = c2_y[c2_i57 + c2_i55];
    }

    c2_i55 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  real_T c2_b_inData[48];
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  real_T c2_u[48];
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i58 = 0;
  for (c2_i59 = 0; c2_i59 < 3; c2_i59++) {
    for (c2_i60 = 0; c2_i60 < 16; c2_i60++) {
      c2_b_inData[c2_i60 + c2_i58] = (*(real_T (*)[48])c2_inData)[c2_i60 +
        c2_i58];
    }

    c2_i58 += 16;
  }

  c2_i61 = 0;
  for (c2_i62 = 0; c2_i62 < 3; c2_i62++) {
    for (c2_i63 = 0; c2_i63 < 16; c2_i63++) {
      c2_u[c2_i63 + c2_i61] = c2_b_inData[c2_i63 + c2_i61];
    }

    c2_i61 += 16;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[48])
{
  real_T c2_dv6[48];
  int32_T c2_i64;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv6, 1, 0, 0U, 1, 0U, 2, 16, 3);
  for (c2_i64 = 0; c2_i64 < 48; c2_i64++) {
    c2_y[c2_i64] = c2_dv6[c2_i64];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_SensorPosNew;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[48];
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_SensorPosNew = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_SensorPosNew), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_SensorPosNew);
  c2_i65 = 0;
  for (c2_i66 = 0; c2_i66 < 3; c2_i66++) {
    for (c2_i67 = 0; c2_i67 < 16; c2_i67++) {
      (*(real_T (*)[48])c2_outData)[c2_i67 + c2_i65] = c2_y[c2_i67 + c2_i65];
    }

    c2_i65 += 16;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  real_T c2_b_inData[32];
  int32_T c2_i71;
  int32_T c2_i72;
  int32_T c2_i73;
  real_T c2_u[32];
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i68 = 0;
  for (c2_i69 = 0; c2_i69 < 2; c2_i69++) {
    for (c2_i70 = 0; c2_i70 < 16; c2_i70++) {
      c2_b_inData[c2_i70 + c2_i68] = (*(real_T (*)[32])c2_inData)[c2_i70 +
        c2_i68];
    }

    c2_i68 += 16;
  }

  c2_i71 = 0;
  for (c2_i72 = 0; c2_i72 < 2; c2_i72++) {
    for (c2_i73 = 0; c2_i73 < 16; c2_i73++) {
      c2_u[c2_i73 + c2_i71] = c2_b_inData[c2_i73 + c2_i71];
    }

    c2_i71 += 16;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[32])
{
  real_T c2_dv7[32];
  int32_T c2_i74;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv7, 1, 0, 0U, 1, 0U, 2, 16, 2);
  for (c2_i74 = 0; c2_i74 < 32; c2_i74++) {
    c2_y[c2_i74] = c2_dv7[c2_i74];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_SensorPos;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[32];
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_SensorPos = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_SensorPos), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_SensorPos);
  c2_i75 = 0;
  for (c2_i76 = 0; c2_i76 < 2; c2_i76++) {
    for (c2_i77 = 0; c2_i77 < 16; c2_i77++) {
      (*(real_T (*)[32])c2_outData)[c2_i77 + c2_i75] = c2_y[c2_i77 + c2_i75];
    }

    c2_i75 += 16;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 42, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sind"), "name", "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343837578U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isfinite"), "name", "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286825976U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286825982U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_rem90"), "name",
                  "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343837578U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rem"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363717452U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286825912U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("cosd"), "name", "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343837578U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), false);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  return c2_y;
}

static real_T c2_sind(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sind(chartInstance, &c2_b_x);
  return c2_b_x;
}

static boolean_T c2_isfinite(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  boolean_T c2_b_b;
  boolean_T c2_b0;
  real_T c2_c_x;
  boolean_T c2_c_b;
  boolean_T c2_b1;
  (void)chartInstance;
  c2_b_x = c2_x;
  c2_b_b = muDoubleScalarIsInf(c2_b_x);
  c2_b0 = !c2_b_b;
  c2_c_x = c2_x;
  c2_c_b = muDoubleScalarIsNaN(c2_c_x);
  c2_b1 = !c2_c_b;
  return c2_b0 && c2_b1;
}

static real_T c2_cosd(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_cosd(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_scalar_eg(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_threshold(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_eml_scalar_eg(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_h_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i78;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i78, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i78;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_i_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_Controller_Pioneer3DXAssembly, const char_T
   *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Controller_Pioneer3DXAssembly), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Controller_Pioneer3DXAssembly);
  return c2_y;
}

static uint8_T c2_j_emlrt_marshallIn
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sind(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T *c2_x)
{
  real_T c2_b_x;
  real_T c2_d5;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_xk;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_absx;
  real_T c2_h_x;
  real_T c2_i_x;
  int8_T c2_n;
  int8_T c2_b_n;
  c2_b_x = *c2_x;
  if (!c2_isfinite(chartInstance, c2_b_x)) {
    c2_d5 = rtNaN;
  } else {
    c2_c_x = c2_b_x;
    c2_d_x = c2_c_x;
    c2_e_x = c2_d_x;
    c2_xk = c2_e_x;
    c2_d_x = muDoubleScalarRem(c2_xk, 360.0);
    c2_f_x = c2_d_x;
    c2_g_x = c2_f_x;
    c2_absx = muDoubleScalarAbs(c2_g_x);
    if (c2_absx > 180.0) {
      if (c2_d_x > 0.0) {
        c2_d_x -= 360.0;
      } else {
        c2_d_x += 360.0;
      }

      c2_h_x = c2_d_x;
      c2_i_x = c2_h_x;
      c2_absx = muDoubleScalarAbs(c2_i_x);
    }

    if (c2_absx <= 45.0) {
      c2_d_x *= 0.017453292519943295;
      c2_n = 0;
    } else if (c2_absx <= 135.0) {
      if (c2_d_x > 0.0) {
        c2_d_x = 0.017453292519943295 * (c2_d_x - 90.0);
        c2_n = 1;
      } else {
        c2_d_x = 0.017453292519943295 * (c2_d_x + 90.0);
        c2_n = -1;
      }
    } else if (c2_d_x > 0.0) {
      c2_d_x = 0.017453292519943295 * (c2_d_x - 180.0);
      c2_n = 2;
    } else {
      c2_d_x = 0.017453292519943295 * (c2_d_x + 180.0);
      c2_n = -2;
    }

    c2_b_n = c2_n;
    c2_b_x = c2_d_x;
    if ((real_T)c2_b_n == 0.0) {
      c2_d5 = muDoubleScalarSin(c2_b_x);
    } else if ((real_T)c2_b_n == 1.0) {
      c2_d5 = muDoubleScalarCos(c2_b_x);
    } else if ((real_T)c2_b_n == -1.0) {
      c2_d5 = -muDoubleScalarCos(c2_b_x);
    } else {
      c2_d5 = -muDoubleScalarSin(c2_b_x);
    }
  }

  *c2_x = c2_d5;
}

static void c2_b_cosd(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct
                      *chartInstance, real_T *c2_x)
{
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_xk;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_absx;
  real_T c2_h_x;
  real_T c2_i_x;
  int8_T c2_n;
  int8_T c2_b_n;
  c2_b_x = *c2_x;
  if (!c2_isfinite(chartInstance, c2_b_x)) {
    *c2_x = rtNaN;
  } else {
    c2_c_x = c2_b_x;
    c2_d_x = c2_c_x;
    c2_e_x = c2_d_x;
    c2_xk = c2_e_x;
    c2_d_x = muDoubleScalarRem(c2_xk, 360.0);
    c2_f_x = c2_d_x;
    c2_g_x = c2_f_x;
    c2_absx = muDoubleScalarAbs(c2_g_x);
    if (c2_absx > 180.0) {
      if (c2_d_x > 0.0) {
        c2_d_x -= 360.0;
      } else {
        c2_d_x += 360.0;
      }

      c2_h_x = c2_d_x;
      c2_i_x = c2_h_x;
      c2_absx = muDoubleScalarAbs(c2_i_x);
    }

    if (c2_absx <= 45.0) {
      c2_d_x *= 0.017453292519943295;
      c2_n = 0;
    } else if (c2_absx <= 135.0) {
      if (c2_d_x > 0.0) {
        c2_d_x = 0.017453292519943295 * (c2_d_x - 90.0);
        c2_n = 1;
      } else {
        c2_d_x = 0.017453292519943295 * (c2_d_x + 90.0);
        c2_n = -1;
      }
    } else if (c2_d_x > 0.0) {
      c2_d_x = 0.017453292519943295 * (c2_d_x - 180.0);
      c2_n = 2;
    } else {
      c2_d_x = 0.017453292519943295 * (c2_d_x + 180.0);
      c2_n = -2;
    }

    c2_b_n = c2_n;
    c2_b_x = c2_d_x;
    if ((real_T)c2_b_n == 0.0) {
      *c2_x = muDoubleScalarCos(c2_b_x);
    } else if ((real_T)c2_b_n == 1.0) {
      *c2_x = -muDoubleScalarSin(c2_b_x);
    } else if ((real_T)c2_b_n == -1.0) {
      *c2_x = muDoubleScalarSin(c2_b_x);
    } else {
      *c2_x = -muDoubleScalarCos(c2_b_x);
    }
  }
}

static void init_dsm_address_info
  (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_Controller_Pioneer3DXAssembly_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(410638684U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1778890143U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(456551622U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(488215668U);
}

mxArray *sf_c2_Controller_Pioneer3DXAssembly_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("nfuwC8ndxMDajDwVxDy9SH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(16);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(16);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Controller_Pioneer3DXAssembly_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_Controller_Pioneer3DXAssembly
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[7],T\"xRange\",},{M[1],M[8],T\"yRange\",},{M[8],M[0],T\"is_active_c2_Controller_Pioneer3DXAssembly\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Controller_Pioneer3DXAssembly_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Controller_Pioneer3DXAssemblyMachineNumber_,
           2,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation
          (_Controller_Pioneer3DXAssemblyMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _Controller_Pioneer3DXAssemblyMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Controller_Pioneer3DXAssemblyMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"rotMat");
          _SFD_SET_DATA_PROPS(3,2,0,1,"xRange");
          _SFD_SET_DATA_PROPS(4,2,0,1,"yRange");
          _SFD_SET_DATA_PROPS(5,10,0,0,"r_range");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,3,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1229);
        _SFD_CV_INIT_EML_FOR(0,1,0,626,648,732);
        _SFD_CV_INIT_EML_FOR(0,1,1,902,924,1014);
        _SFD_CV_INIT_EML_FOR(0,1,2,1077,1090,1162);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        {
          real_T *c2_x;
          real_T *c2_y;
          real_T (*c2_rotMat)[9];
          real_T (*c2_xRange)[16];
          real_T (*c2_yRange)[16];
          c2_yRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_xRange = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_rotMat = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
          c2_y = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_x = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_x);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_y);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_rotMat);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_xRange);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_yRange);
          _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c2_r_range);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Controller_Pioneer3DXAssemblyMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "xU27y96WtTY23iucuDiKSC";
}

static void sf_opaque_initialize_c2_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
  initialize_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  enable_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  disable_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  sf_gateway_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_Controller_Pioneer3DXAssembly
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_Controller_Pioneer3DXAssembly();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_Controller_Pioneer3DXAssembly(SimStruct*
  S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c2_Controller_Pioneer3DXAssembly();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_Controller_Pioneer3DXAssembly
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c2_Controller_Pioneer3DXAssembly(S);
}

static void sf_opaque_set_sim_state_c2_Controller_Pioneer3DXAssembly(SimStruct*
  S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_Controller_Pioneer3DXAssembly(S, st);
}

static void sf_opaque_terminate_c2_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Controller_Pioneer3DXAssembly_optimization_info();
    }

    finalize_c2_Controller_Pioneer3DXAssembly
      ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Controller_Pioneer3DXAssembly
    ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_Controller_Pioneer3DXAssembly
      ((SFc2_Controller_Pioneer3DXAssemblyInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  /* Actual parameters from chart:
     r_range
   */
  const char_T *rtParamNames[] = { "r_range" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for r_range*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Controller_Pioneer3DXAssembly_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(871289717U));
  ssSetChecksum1(S,(2877147831U));
  ssSetChecksum2(S,(1998220361U));
  ssSetChecksum3(S,(1804438237U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct *)utMalloc
    (sizeof(SFc2_Controller_Pioneer3DXAssemblyInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc2_Controller_Pioneer3DXAssemblyInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_Controller_Pioneer3DXAssembly_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Controller_Pioneer3DXAssembly(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Controller_Pioneer3DXAssembly(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Controller_Pioneer3DXAssembly(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Controller_Pioneer3DXAssembly_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
