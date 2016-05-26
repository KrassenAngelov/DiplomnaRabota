/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Controller_Pioneer3DXAssembly_sfun.h"
#include "c1_Controller_Pioneer3DXAssembly.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Controller_Pioneer3DXAssembly_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c1_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c1_IN_Modes                    ((uint8_T)1U)
#define c1_IN_DetermineLeftRight       ((uint8_T)1U)
#define c1_IN_GoStraight               ((uint8_T)2U)
#define c1_IN_ReverseABit              ((uint8_T)3U)
#define c1_IN_TurnLeftInPlace          ((uint8_T)4U)
#define c1_IN_TurnLeftTight            ((uint8_T)5U)
#define c1_IN_TurnLeftWide             ((uint8_T)6U)
#define c1_IN_TurnRightInPlace         ((uint8_T)7U)
#define c1_IN_TurnRightTight           ((uint8_T)8U)
#define c1_IN_TurnRightWide            ((uint8_T)9U)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_d_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_e_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_f_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_h_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_i_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_j_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_k_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_l_debug_family_names[2] = { "nargin", "nargout" };

static const char * c1_m_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_n_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_o_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_p_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_q_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_r_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_s_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_t_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_u_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_v_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_w_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_x_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_y_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_ab_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_bb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_cb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_db_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_eb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_fb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_gb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_hb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_ib_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_jb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_kb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_lb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c1_mb_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static boolean_T c1_dataWrittenToVector[7];

/* Function Declarations */
static void initialize_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void initialize_params_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void enable_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void disable_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void set_sim_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_st);
static void c1_set_sim_state_side_effects_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void finalize_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void sf_gateway_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void initSimStructsc1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);
static void c1_Modes(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
                     *chartInstance);
static void c1_TurnLeftWide(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static void c1_TurnRightInPlace(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static void c1_TurnRightTight(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static void c1_TurnRightWide(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static boolean_T c1_all(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x);
static void c1_isequal(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static boolean_T c1_any(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x);
static boolean_T c1_b_all(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x[2]);
static void c1_b_isequal(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance);
static boolean_T c1_b_any(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x[2]);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_nargout, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static boolean_T c1_c_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_sf_internal_predicateOutput, const char_T *c1_identifier);
static boolean_T c1_d_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_emlrt_marshallOut(const char * c1_u);
static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_e_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_f_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_tp_Modes, const char_T *c1_identifier);
static uint8_T c1_g_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_h_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_frontSensors, const char_T *c1_identifier, boolean_T c1_y[2]);
static void c1_i_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, boolean_T c1_y[2]);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_j_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_setSimStateSideEffectsInfo, const char_T *c1_identifier);
static const mxArray *c1_k_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_updateDataWrittenToVector
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, uint32_T
   c1_vectorIndex);
static void c1_errorIfDataNotWrittenToFcn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, uint32_T
   c1_vectorIndex, uint32_T c1_dataNumber);
static void init_dsm_address_info
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_doSetSimStateSideEffects = 0U;
  chartInstance->c1_setSimStateSideEffectsInfo = NULL;
  chartInstance->c1_is_Modes = c1_IN_NO_ACTIVE_CHILD;
  chartInstance->c1_tp_Modes = 0U;
  chartInstance->c1_tp_DetermineLeftRight = 0U;
  chartInstance->c1_tp_GoStraight = 0U;
  chartInstance->c1_tp_ReverseABit = 0U;
  chartInstance->c1_temporalCounter_i1 = 0.0;
  chartInstance->c1_tp_TurnLeftInPlace = 0U;
  chartInstance->c1_tp_TurnLeftTight = 0U;
  chartInstance->c1_tp_TurnLeftWide = 0U;
  chartInstance->c1_tp_TurnRightInPlace = 0U;
  chartInstance->c1_tp_TurnRightTight = 0U;
  chartInstance->c1_tp_TurnRightWide = 0U;
  chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly = 0U;
  chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly = c1_IN_NO_ACTIVE_CHILD;
  chartInstance->c1_presentTime = 0.0;
  chartInstance->c1_elapsedTime = 0.0;
  chartInstance->c1_previousTime = 0.0;
}

static void initialize_params_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  real_T c1_d0;
  sf_mex_import_named("v0", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      &c1_d0, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c1_v0 = c1_d0;
}

static void enable_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_presentTime = _sfTime_;
  chartInstance->c1_previousTime = chartInstance->c1_presentTime;
}

static void disable_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_presentTime = _sfTime_;
  chartInstance->c1_elapsedTime = chartInstance->c1_presentTime -
    chartInstance->c1_previousTime;
  chartInstance->c1_previousTime = chartInstance->c1_presentTime;
  chartInstance->c1_temporalCounter_i1 += chartInstance->c1_elapsedTime;
}

static void c1_update_debugger_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  uint32_T c1_prevAniVal;
  c1_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly == c1_IN_Modes) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnLeftWide) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 6U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 6U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_GoStraight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_DetermineLeftRight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnRightInPlace) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 7U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 7U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnLeftInPlace) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnRightTight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 8U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 8U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnLeftTight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_TurnRightWide) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 9U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 9U, chartInstance->c1_sfEvent);
  }

  if (chartInstance->c1_is_Modes == c1_IN_ReverseABit) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
  }

  _SFD_SET_ANIMATION(c1_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static const mxArray *get_sim_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  int32_T c1_i0;
  boolean_T c1_c_u[2];
  const mxArray *c1_d_y = NULL;
  int32_T c1_i1;
  boolean_T c1_d_u[2];
  const mxArray *c1_e_y = NULL;
  boolean_T c1_c_hoistedGlobal;
  boolean_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  int32_T c1_i2;
  boolean_T c1_f_u[2];
  const mxArray *c1_g_y = NULL;
  boolean_T c1_d_hoistedGlobal;
  boolean_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  uint8_T c1_e_hoistedGlobal;
  uint8_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  uint8_T c1_f_hoistedGlobal;
  uint8_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  uint8_T c1_g_hoistedGlobal;
  uint8_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_k_u;
  const mxArray *c1_l_y = NULL;
  real_T c1_i_hoistedGlobal;
  real_T c1_l_u;
  const mxArray *c1_m_y = NULL;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(12, 1), false);
  c1_hoistedGlobal = *c1_LeftVel;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_RightVel;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  for (c1_i0 = 0; c1_i0 < 2; c1_i0++) {
    c1_c_u[c1_i0] = chartInstance->c1_frontSensors[c1_i0];
  }

  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_c_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  for (c1_i1 = 0; c1_i1 < 2; c1_i1++) {
    c1_d_u[c1_i1] = chartInstance->c1_leftMiddleSensors[c1_i1];
  }

  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", c1_d_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_c_hoistedGlobal = chartInstance->c1_leftMostSensors;
  c1_e_u = c1_c_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  for (c1_i2 = 0; c1_i2 < 2; c1_i2++) {
    c1_f_u[c1_i2] = chartInstance->c1_rightMiddleSensors[c1_i2];
  }

  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_f_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_d_hoistedGlobal = chartInstance->c1_rightMostSensors;
  c1_g_u = c1_d_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_e_hoistedGlobal =
    chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly;
  c1_h_u = c1_e_hoistedGlobal;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_f_hoistedGlobal = chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly;
  c1_i_u = c1_f_hoistedGlobal;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_g_hoistedGlobal = chartInstance->c1_is_Modes;
  c1_j_u = c1_g_hoistedGlobal;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  c1_h_hoistedGlobal = chartInstance->c1_temporalCounter_i1;
  c1_k_u = c1_h_hoistedGlobal;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 10, c1_l_y);
  c1_i_hoistedGlobal = chartInstance->c1_previousTime;
  c1_l_u = c1_i_hoistedGlobal;
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 11, c1_m_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_st)
{
  const mxArray *c1_u;
  boolean_T c1_bv0[2];
  int32_T c1_i3;
  boolean_T c1_bv1[2];
  int32_T c1_i4;
  boolean_T c1_bv2[2];
  int32_T c1_i5;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_u = sf_mex_dup(c1_st);
  *c1_LeftVel = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 0)), "LeftVel");
  *c1_RightVel = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 1)), "RightVel");
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)),
                        "frontSensors", c1_bv0);
  for (c1_i3 = 0; c1_i3 < 2; c1_i3++) {
    chartInstance->c1_frontSensors[c1_i3] = c1_bv0[c1_i3];
  }

  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 3)),
                        "leftMiddleSensors", c1_bv1);
  for (c1_i4 = 0; c1_i4 < 2; c1_i4++) {
    chartInstance->c1_leftMiddleSensors[c1_i4] = c1_bv1[c1_i4];
  }

  chartInstance->c1_leftMostSensors = c1_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 4)), "leftMostSensors");
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 5)),
                        "rightMiddleSensors", c1_bv2);
  for (c1_i5 = 0; c1_i5 < 2; c1_i5++) {
    chartInstance->c1_rightMiddleSensors[c1_i5] = c1_bv2[c1_i5];
  }

  chartInstance->c1_rightMostSensors = c1_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 6)), "rightMostSensors");
  chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly =
    c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 7)),
    "is_active_c1_Controller_Pioneer3DXAssembly");
  chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly = c1_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 8)),
     "is_c1_Controller_Pioneer3DXAssembly");
  chartInstance->c1_is_Modes = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 9)), "is_Modes");
  chartInstance->c1_temporalCounter_i1 = c1_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 10)), "temporalCounter_i1");
  chartInstance->c1_previousTime = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 11)), "previousTime");
  sf_mex_assign(&chartInstance->c1_setSimStateSideEffectsInfo,
                c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 12)), "setSimStateSideEffectsInfo"), true);
  sf_mex_destroy(&c1_u);
  chartInstance->c1_doSetSimStateSideEffects = 1U;
  c1_update_debugger_state_c1_Controller_Pioneer3DXAssembly(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void c1_set_sim_state_side_effects_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  if (chartInstance->c1_doSetSimStateSideEffects != 0) {
    if (chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly == c1_IN_Modes) {
      chartInstance->c1_tp_Modes = 1U;
    } else {
      chartInstance->c1_tp_Modes = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_DetermineLeftRight) {
      chartInstance->c1_tp_DetermineLeftRight = 1U;
    } else {
      chartInstance->c1_tp_DetermineLeftRight = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_GoStraight) {
      chartInstance->c1_tp_GoStraight = 1U;
    } else {
      chartInstance->c1_tp_GoStraight = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_ReverseABit) {
      chartInstance->c1_tp_ReverseABit = 1U;
      if (sf_mex_sub(chartInstance->c1_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 5) == 0.0) {
        chartInstance->c1_temporalCounter_i1 = 0.0;
      }
    } else {
      chartInstance->c1_tp_ReverseABit = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnLeftInPlace) {
      chartInstance->c1_tp_TurnLeftInPlace = 1U;
    } else {
      chartInstance->c1_tp_TurnLeftInPlace = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnLeftTight) {
      chartInstance->c1_tp_TurnLeftTight = 1U;
    } else {
      chartInstance->c1_tp_TurnLeftTight = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnLeftWide) {
      chartInstance->c1_tp_TurnLeftWide = 1U;
    } else {
      chartInstance->c1_tp_TurnLeftWide = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnRightInPlace) {
      chartInstance->c1_tp_TurnRightInPlace = 1U;
    } else {
      chartInstance->c1_tp_TurnRightInPlace = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnRightTight) {
      chartInstance->c1_tp_TurnRightTight = 1U;
    } else {
      chartInstance->c1_tp_TurnRightTight = 0U;
    }

    if (chartInstance->c1_is_Modes == c1_IN_TurnRightWide) {
      chartInstance->c1_tp_TurnRightWide = 1U;
    } else {
      chartInstance->c1_tp_TurnRightWide = 0U;
    }

    chartInstance->c1_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  sf_mex_destroy(&chartInstance->c1_setSimStateSideEffectsInfo);
}

static void sf_gateway_c1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  int32_T c1_i6;
  int32_T c1_i7;
  int32_T c1_i8;
  int32_T c1_i9;
  int32_T c1_i10;
  uint32_T c1_debug_family_var_map[2];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 0.0;
  int32_T c1_i11;
  int32_T c1_i12;
  int32_T c1_i13;
  int32_T c1_i14;
  int32_T c1_i15;
  int32_T c1_i16;
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  real_T (*c1_SonarSensors)[16];
  real_T (*c1_SensorRanges)[16];
  c1_SensorRanges = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c1_SonarSensors = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 0);
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_set_sim_state_side_effects_c1_Controller_Pioneer3DXAssembly(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_presentTime = _sfTime_;
  chartInstance->c1_elapsedTime = chartInstance->c1_presentTime -
    chartInstance->c1_previousTime;
  chartInstance->c1_previousTime = chartInstance->c1_presentTime;
  chartInstance->c1_temporalCounter_i1 += chartInstance->c1_elapsedTime;
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
  _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
  for (c1_i6 = 0; c1_i6 < 16; c1_i6++) {
    _SFD_DATA_RANGE_CHECK((*c1_SonarSensors)[c1_i6], 2U);
  }

  for (c1_i7 = 0; c1_i7 < 2; c1_i7++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_frontSensors[c1_i7], 3U);
  }

  for (c1_i8 = 0; c1_i8 < 16; c1_i8++) {
    _SFD_DATA_RANGE_CHECK((*c1_SensorRanges)[c1_i8], 4U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c1_v0, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMostSensors, 6U);
  for (c1_i9 = 0; c1_i9 < 2; c1_i9++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMiddleSensors[c1_i9], 7U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMostSensors, 8U);
  for (c1_i10 = 0; c1_i10 < 2; c1_i10++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMiddleSensors[c1_i10],
                          9U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  if (chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_active_c1_Controller_Pioneer3DXAssembly = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_c1_Controller_Pioneer3DXAssembly = c1_IN_Modes;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_Modes = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_debug_family_names,
      c1_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    for (c1_i11 = 0; c1_i11 < 2; c1_i11++) {
      chartInstance->c1_frontSensors[c1_i11] = ((*c1_SonarSensors)[c1_i11 + 3] <
        (*c1_SensorRanges)[c1_i11 + 3]);
    }

    c1_updateDataWrittenToVector(chartInstance, 2U);
    for (c1_i12 = 0; c1_i12 < 2; c1_i12++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_frontSensors[c1_i12], 3U);
    }

    chartInstance->c1_leftMostSensors = ((*c1_SonarSensors)[0] <
      (*c1_SensorRanges)[0]);
    c1_updateDataWrittenToVector(chartInstance, 3U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMostSensors, 6U);
    for (c1_i13 = 0; c1_i13 < 2; c1_i13++) {
      chartInstance->c1_leftMiddleSensors[c1_i13] = ((*c1_SonarSensors)[c1_i13 +
        1] < (*c1_SensorRanges)[c1_i13 + 1]);
    }

    c1_updateDataWrittenToVector(chartInstance, 4U);
    for (c1_i14 = 0; c1_i14 < 2; c1_i14++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMiddleSensors[c1_i14],
                            7U);
    }

    for (c1_i15 = 0; c1_i15 < 2; c1_i15++) {
      chartInstance->c1_rightMiddleSensors[c1_i15] = ((*c1_SonarSensors)[c1_i15
        + 5] < (*c1_SensorRanges)[c1_i15 + 5]);
    }

    c1_updateDataWrittenToVector(chartInstance, 6U);
    for (c1_i16 = 0; c1_i16 < 2; c1_i16++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMiddleSensors[c1_i16],
                            9U);
    }

    chartInstance->c1_rightMostSensors = ((*c1_SonarSensors)[7] <
      (*c1_SensorRanges)[7]);
    c1_updateDataWrittenToVector(chartInstance, 5U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMostSensors, 8U);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_Modes = c1_IN_ReverseABit;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
    chartInstance->c1_temporalCounter_i1 = 0.0;
    chartInstance->c1_tp_ReverseABit = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_k_debug_family_names,
      c1_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = -chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = -chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    c1_Modes(chartInstance);
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_Controller_Pioneer3DXAssemblyMachineNumber_, chartInstance->chartNumber,
     chartInstance->instanceNumber);
}

static void initSimStructsc1_Controller_Pioneer3DXAssembly
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_Modes(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
                     *chartInstance)
{
  uint32_T c1_debug_family_var_map[2];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 0.0;
  int32_T c1_i17;
  int32_T c1_i18;
  int32_T c1_i19;
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  uint32_T c1_b_debug_family_var_map[3];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 1.0;
  boolean_T c1_out;
  real_T c1_c_nargin = 0.0;
  real_T c1_c_nargout = 1.0;
  boolean_T c1_b_out;
  int32_T c1_i23;
  boolean_T c1_bv3[2];
  real_T c1_d_nargin = 0.0;
  real_T c1_d_nargout = 1.0;
  boolean_T c1_c_out;
  int32_T c1_i24;
  boolean_T c1_bv4[2];
  real_T c1_e_nargin = 0.0;
  real_T c1_e_nargout = 1.0;
  boolean_T c1_d_out;
  int32_T c1_i25;
  boolean_T c1_bv5[2];
  real_T c1_f_nargin = 0.0;
  real_T c1_f_nargout = 1.0;
  boolean_T c1_e_out;
  real_T c1_g_nargin = 0.0;
  real_T c1_g_nargout = 1.0;
  boolean_T c1_f_out;
  real_T c1_h_nargin = 0.0;
  real_T c1_h_nargout = 0.0;
  real_T c1_i_nargin = 0.0;
  real_T c1_i_nargout = 1.0;
  boolean_T c1_g_out;
  real_T c1_j_nargin = 0.0;
  real_T c1_j_nargout = 0.0;
  real_T c1_k_nargin = 0.0;
  real_T c1_k_nargout = 0.0;
  real_T c1_l_nargin = 0.0;
  real_T c1_l_nargout = 1.0;
  boolean_T c1_h_out;
  int32_T c1_i26;
  boolean_T c1_bv6[2];
  real_T c1_m_nargin = 0.0;
  real_T c1_m_nargout = 0.0;
  real_T c1_n_nargin = 0.0;
  real_T c1_n_nargout = 0.0;
  real_T c1_o_nargin = 0.0;
  real_T c1_o_nargout = 1.0;
  boolean_T c1_i_out;
  int32_T c1_i27;
  boolean_T c1_bv7[2];
  real_T c1_p_nargin = 0.0;
  real_T c1_p_nargout = 0.0;
  real_T c1_q_nargin = 0.0;
  real_T c1_q_nargout = 0.0;
  real_T c1_hoistedGlobal;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  real_T (*c1_SonarSensors)[16];
  real_T (*c1_SensorRanges)[16];
  c1_SensorRanges = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c1_SonarSensors = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 0);
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  for (c1_i17 = 0; c1_i17 < 2; c1_i17++) {
    chartInstance->c1_frontSensors[c1_i17] = ((*c1_SonarSensors)[c1_i17 + 3] < (*
      c1_SensorRanges)[c1_i17 + 3]);
  }

  c1_updateDataWrittenToVector(chartInstance, 2U);
  for (c1_i18 = 0; c1_i18 < 2; c1_i18++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_frontSensors[c1_i18], 3U);
  }

  chartInstance->c1_leftMostSensors = ((*c1_SonarSensors)[0] < (*c1_SensorRanges)
    [0]);
  c1_updateDataWrittenToVector(chartInstance, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMostSensors, 6U);
  for (c1_i19 = 0; c1_i19 < 2; c1_i19++) {
    chartInstance->c1_leftMiddleSensors[c1_i19] = ((*c1_SonarSensors)[c1_i19 + 1]
      < (*c1_SensorRanges)[c1_i19 + 1]);
  }

  c1_updateDataWrittenToVector(chartInstance, 4U);
  for (c1_i20 = 0; c1_i20 < 2; c1_i20++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_leftMiddleSensors[c1_i20],
                          7U);
  }

  for (c1_i21 = 0; c1_i21 < 2; c1_i21++) {
    chartInstance->c1_rightMiddleSensors[c1_i21] = ((*c1_SonarSensors)[c1_i21 +
      5] < (*c1_SensorRanges)[c1_i21 + 5]);
  }

  c1_updateDataWrittenToVector(chartInstance, 6U);
  for (c1_i22 = 0; c1_i22 < 2; c1_i22++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMiddleSensors[c1_i22],
                          9U);
  }

  chartInstance->c1_rightMostSensors = ((*c1_SonarSensors)[7] <
    (*c1_SensorRanges)[7]);
  c1_updateDataWrittenToVector(chartInstance, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c1_rightMostSensors, 8U);
  _SFD_SYMBOL_SCOPE_POP();
  switch (chartInstance->c1_is_Modes) {
   case c1_IN_DetermineLeftRight:
    CV_STATE_EVAL(0, 0, 1);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 12U,
                 chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_m_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    c1_out = CV_EML_IF(12, 0, 0, (*c1_SonarSensors)[3] > (*c1_SonarSensors)[4]);
    _SFD_SYMBOL_SCOPE_POP();
    if (c1_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 12U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_DetermineLeftRight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_TurnRightInPlace;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 7U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_TurnRightInPlace = 1U;
    } else {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 13U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_DetermineLeftRight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_TurnLeftInPlace;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_TurnLeftInPlace = 1U;
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c1_sfEvent);
    break;

   case c1_IN_GoStraight:
    CV_STATE_EVAL(0, 0, 2);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_s_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    c1_errorIfDataNotWrittenToFcn(chartInstance, 2U, 3U);
    for (c1_i23 = 0; c1_i23 < 2; c1_i23++) {
      c1_bv3[c1_i23] = ((real_T)chartInstance->c1_frontSensors[c1_i23] > 0.0);
    }

    c1_b_out = CV_EML_IF(1, 0, 0, c1_b_any(chartInstance, c1_bv3));
    _SFD_SYMBOL_SCOPE_POP();
    if (c1_b_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_GoStraight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_DetermineLeftRight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_DetermineLeftRight = 1U;
    } else {
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 9U,
                   chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_q_debug_family_names,
        c1_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_out, 2U, c1_b_sf_marshallOut,
        c1_b_sf_marshallIn);
      c1_errorIfDataNotWrittenToFcn(chartInstance, 4U, 7U);
      for (c1_i24 = 0; c1_i24 < 2; c1_i24++) {
        c1_bv4[c1_i24] = ((real_T)chartInstance->c1_leftMiddleSensors[c1_i24] >
                          0.0);
      }

      c1_c_out = CV_EML_IF(9, 0, 0, c1_b_any(chartInstance, c1_bv4));
      _SFD_SYMBOL_SCOPE_POP();
      if (c1_c_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 9U, chartInstance->c1_sfEvent);
        chartInstance->c1_tp_GoStraight = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
        chartInstance->c1_is_Modes = c1_IN_TurnRightTight;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 8U, chartInstance->c1_sfEvent);
        chartInstance->c1_tp_TurnRightTight = 1U;
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 3U,
                     chartInstance->c1_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_v_debug_family_names,
          c1_b_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargin, 0U, c1_sf_marshallOut,
          c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_nargout, 1U,
          c1_sf_marshallOut, c1_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_d_out, 2U, c1_b_sf_marshallOut,
          c1_b_sf_marshallIn);
        c1_errorIfDataNotWrittenToFcn(chartInstance, 6U, 9U);
        for (c1_i25 = 0; c1_i25 < 2; c1_i25++) {
          c1_bv5[c1_i25] = ((real_T)chartInstance->c1_rightMiddleSensors[c1_i25]
                            > 0.0);
        }

        c1_d_out = CV_EML_IF(3, 0, 0, c1_b_any(chartInstance, c1_bv5));
        _SFD_SYMBOL_SCOPE_POP();
        if (c1_d_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
          chartInstance->c1_tp_GoStraight = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
          chartInstance->c1_is_Modes = c1_IN_TurnLeftTight;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c1_sfEvent);
          chartInstance->c1_tp_TurnLeftTight = 1U;
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 7U,
                       chartInstance->c1_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_o_debug_family_names,
            c1_b_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargin, 0U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_nargout, 1U,
            c1_sf_marshallOut, c1_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_e_out, 2U,
            c1_b_sf_marshallOut, c1_b_sf_marshallIn);
          c1_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
          c1_e_out = CV_EML_IF(7, 0, 0, c1_any(chartInstance, (real_T)
            chartInstance->c1_leftMostSensors > 0.0));
          _SFD_SYMBOL_SCOPE_POP();
          if (c1_e_out) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, chartInstance->c1_sfEvent);
            chartInstance->c1_tp_GoStraight = 0U;
            _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
            chartInstance->c1_is_Modes = c1_IN_TurnRightWide;
            _SFD_CS_CALL(STATE_ACTIVE_TAG, 9U, chartInstance->c1_sfEvent);
            chartInstance->c1_tp_TurnRightWide = 1U;
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 5U,
                         chartInstance->c1_sfEvent);
            _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_x_debug_family_names,
              c1_b_debug_family_var_map);
            _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargin, 0U,
              c1_sf_marshallOut, c1_sf_marshallIn);
            _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_nargout, 1U,
              c1_sf_marshallOut, c1_sf_marshallIn);
            _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_f_out, 2U,
              c1_b_sf_marshallOut, c1_b_sf_marshallIn);
            c1_errorIfDataNotWrittenToFcn(chartInstance, 5U, 8U);
            c1_f_out = CV_EML_IF(5, 0, 0, c1_any(chartInstance, (real_T)
              chartInstance->c1_rightMostSensors > 0.0));
            _SFD_SYMBOL_SCOPE_POP();
            if (c1_f_out) {
              _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, chartInstance->c1_sfEvent);
              chartInstance->c1_tp_GoStraight = 0U;
              _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
              chartInstance->c1_is_Modes = c1_IN_TurnLeftWide;
              _SFD_CS_CALL(STATE_ACTIVE_TAG, 6U, chartInstance->c1_sfEvent);
              chartInstance->c1_tp_TurnLeftWide = 1U;
            } else {
              _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                           chartInstance->c1_sfEvent);
              _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_e_debug_family_names,
                c1_debug_family_var_map);
              _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargin, 0U,
                c1_sf_marshallOut, c1_sf_marshallIn);
              _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_nargout, 1U,
                c1_sf_marshallOut, c1_sf_marshallIn);
              *c1_RightVel = chartInstance->c1_v0;
              c1_updateDataWrittenToVector(chartInstance, 1U);
              _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
              *c1_LeftVel = chartInstance->c1_v0;
              c1_updateDataWrittenToVector(chartInstance, 0U);
              _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
              _SFD_SYMBOL_SCOPE_POP();
            }
          }
        }
      }
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c1_sfEvent);
    break;

   case c1_IN_ReverseABit:
    CV_STATE_EVAL(0, 0, 3);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 14U,
                 chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_y_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    c1_g_out = CV_EML_IF(14, 0, 0, chartInstance->c1_temporalCounter_i1 +
                         (_sfTime_ - chartInstance->c1_previousTime) >= 3.0);
    _SFD_SYMBOL_SCOPE_POP();
    if (c1_g_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 14U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_ReverseABit = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_j_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      *c1_LeftVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 3U,
                   chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_l_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_k_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_k_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = -chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      *c1_LeftVel = -chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c1_sfEvent);
    break;

   case c1_IN_TurnLeftInPlace:
    CV_STATE_EVAL(0, 0, 4);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 11U,
                 chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_t_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_l_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_l_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_h_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    c1_errorIfDataNotWrittenToFcn(chartInstance, 2U, 3U);
    for (c1_i26 = 0; c1_i26 < 2; c1_i26++) {
      c1_bv6[c1_i26] = ((real_T)chartInstance->c1_frontSensors[c1_i26] < 1.0);
    }

    c1_h_out = CV_EML_IF(11, 0, 0, c1_b_all(chartInstance, c1_bv6));
    _SFD_SYMBOL_SCOPE_POP();
    if (c1_h_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 11U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_TurnLeftInPlace = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_m_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_m_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      *c1_LeftVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                   chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_g_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_n_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_n_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      *c1_LeftVel = -chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c1_sfEvent);
    break;

   case c1_IN_TurnLeftTight:
    CV_STATE_EVAL(0, 0, 5);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 4U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_u_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_o_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_o_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i_out, 2U, c1_b_sf_marshallOut,
      c1_b_sf_marshallIn);
    c1_errorIfDataNotWrittenToFcn(chartInstance, 6U, 9U);
    for (c1_i27 = 0; c1_i27 < 2; c1_i27++) {
      c1_bv7[c1_i27] = ((real_T)chartInstance->c1_rightMiddleSensors[c1_i27] <
                        1.0);
    }

    c1_i_out = CV_EML_IF(4, 0, 0, c1_b_all(chartInstance, c1_bv7));
    _SFD_SYMBOL_SCOPE_POP();
    if (c1_i_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_TurnLeftTight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c1_sfEvent);
      chartInstance->c1_is_Modes = c1_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
      chartInstance->c1_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_p_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_p_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      *c1_LeftVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 5U,
                   chartInstance->c1_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_i_debug_family_names,
        c1_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_q_nargin, 0U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_q_nargout, 1U, c1_sf_marshallOut,
        c1_sf_marshallIn);
      *c1_RightVel = chartInstance->c1_v0;
      c1_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
      c1_hoistedGlobal = chartInstance->c1_v0;
      c1_A = c1_hoistedGlobal;
      c1_x = c1_A;
      c1_b_x = c1_x;
      c1_c_x = c1_b_x;
      *c1_LeftVel = c1_c_x / 5.0;
      c1_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c1_sfEvent);
    break;

   case c1_IN_TurnLeftWide:
    CV_STATE_EVAL(0, 0, 6);
    c1_TurnLeftWide(chartInstance);
    break;

   case c1_IN_TurnRightInPlace:
    CV_STATE_EVAL(0, 0, 7);
    c1_TurnRightInPlace(chartInstance);
    break;

   case c1_IN_TurnRightTight:
    CV_STATE_EVAL(0, 0, 8);
    c1_TurnRightTight(chartInstance);
    break;

   case c1_IN_TurnRightWide:
    CV_STATE_EVAL(0, 0, 9);
    c1_TurnRightWide(chartInstance);
    break;

   default:
    CV_STATE_EVAL(0, 0, 0);
    chartInstance->c1_is_Modes = c1_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c1_sfEvent);
    break;
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void c1_TurnLeftWide(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  boolean_T c1_out;
  uint32_T c1_b_debug_family_var_map[2];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T c1_c_nargin = 0.0;
  real_T c1_c_nargout = 0.0;
  real_T c1_hoistedGlobal;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 6U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_w_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  c1_errorIfDataNotWrittenToFcn(chartInstance, 5U, 8U);
  c1_out = CV_EML_IF(6, 0, 0, c1_all(chartInstance, (real_T)
    chartInstance->c1_rightMostSensors < 1.0));
  _SFD_SYMBOL_SCOPE_POP();
  if (c1_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_TurnLeftWide = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 6U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_Modes = c1_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_c_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    c1_hoistedGlobal = chartInstance->c1_v0;
    c1_A = c1_hoistedGlobal;
    c1_x = c1_A;
    c1_b_x = c1_x;
    c1_c_x = c1_b_x;
    *c1_LeftVel = c1_c_x / 2.0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c1_sfEvent);
}

static void c1_TurnRightInPlace(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  boolean_T c1_out;
  int32_T c1_i28;
  boolean_T c1_bv8[2];
  uint32_T c1_b_debug_family_var_map[2];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T c1_c_nargin = 0.0;
  real_T c1_c_nargout = 0.0;
  real_T *c1_RightVel;
  real_T *c1_LeftVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 2U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_r_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  c1_errorIfDataNotWrittenToFcn(chartInstance, 2U, 3U);
  for (c1_i28 = 0; c1_i28 < 2; c1_i28++) {
    c1_bv8[c1_i28] = ((real_T)chartInstance->c1_frontSensors[c1_i28] < 1.0);
  }

  c1_out = CV_EML_IF(2, 0, 0, c1_b_all(chartInstance, c1_bv8));
  _SFD_SYMBOL_SCOPE_POP();
  if (c1_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_TurnRightInPlace = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 7U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_Modes = c1_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 7U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_f_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = -chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, chartInstance->c1_sfEvent);
}

static void c1_TurnRightTight(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  boolean_T c1_out;
  int32_T c1_i29;
  boolean_T c1_bv9[2];
  uint32_T c1_b_debug_family_var_map[2];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T c1_c_nargin = 0.0;
  real_T c1_c_nargout = 0.0;
  real_T c1_hoistedGlobal;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 10U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_p_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  c1_errorIfDataNotWrittenToFcn(chartInstance, 4U, 7U);
  for (c1_i29 = 0; c1_i29 < 2; c1_i29++) {
    c1_bv9[c1_i29] = ((real_T)chartInstance->c1_leftMiddleSensors[c1_i29] < 1.0);
  }

  c1_out = CV_EML_IF(10, 0, 0, c1_b_all(chartInstance, c1_bv9));
  _SFD_SYMBOL_SCOPE_POP();
  if (c1_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 10U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_TurnRightTight = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 8U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_Modes = c1_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 8U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_h_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    c1_hoistedGlobal = chartInstance->c1_v0;
    c1_A = c1_hoistedGlobal;
    c1_x = c1_A;
    c1_b_x = c1_x;
    c1_c_x = c1_b_x;
    *c1_RightVel = c1_c_x / 5.0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 8U, chartInstance->c1_sfEvent);
}

static void c1_TurnRightWide(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  boolean_T c1_out;
  uint32_T c1_b_debug_family_var_map[2];
  real_T c1_b_nargin = 0.0;
  real_T c1_b_nargout = 0.0;
  real_T c1_c_nargin = 0.0;
  real_T c1_c_nargout = 0.0;
  real_T c1_hoistedGlobal;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T *c1_LeftVel;
  real_T *c1_RightVel;
  c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 8U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c1_n_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_out, 2U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  c1_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
  c1_out = CV_EML_IF(8, 0, 0, c1_all(chartInstance, (real_T)
    chartInstance->c1_leftMostSensors < 1.0));
  _SFD_SYMBOL_SCOPE_POP();
  if (c1_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_TurnRightWide = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 9U, chartInstance->c1_sfEvent);
    chartInstance->c1_is_Modes = c1_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c1_sfEvent);
    chartInstance->c1_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_d_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    *c1_RightVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 9U, chartInstance->c1_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c1_j_debug_family_names,
      c1_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargin, 0U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c_nargout, 1U, c1_sf_marshallOut,
      c1_sf_marshallIn);
    c1_hoistedGlobal = chartInstance->c1_v0;
    c1_A = c1_hoistedGlobal;
    c1_x = c1_A;
    c1_b_x = c1_x;
    c1_c_x = c1_b_x;
    *c1_RightVel = c1_c_x / 2.0;
    c1_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c1_RightVel, 1U);
    *c1_LeftVel = chartInstance->c1_v0;
    c1_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c1_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 9U, chartInstance->c1_sfEvent);
}

static boolean_T c1_all(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x)
{
  boolean_T c1_b_x;
  c1_b_x = c1_x;
  c1_isequal(chartInstance);
  return (real_T)c1_b_x != 0.0;
}

static void c1_isequal(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c1_any(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x)
{
  boolean_T c1_b_x;
  boolean_T c1_b0;
  c1_b_x = c1_x;
  c1_isequal(chartInstance);
  c1_b0 = ((real_T)c1_b_x == 0.0);
  return !c1_b0;
}

static boolean_T c1_b_all(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x[2])
{
  boolean_T c1_y;
  int32_T c1_k;
  real_T c1_b_k;
  boolean_T exitg1;
  c1_b_isequal(chartInstance);
  c1_y = true;
  c1_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c1_k < 2)) {
    c1_b_k = 1.0 + (real_T)c1_k;
    if ((real_T)c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
         ("", c1_b_k), 1, 2, 1, 0) - 1] == 0.0) {
      c1_y = false;
      exitg1 = true;
    } else {
      c1_k++;
    }
  }

  return c1_y;
}

static void c1_b_isequal(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c1_b_any(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
  *chartInstance, boolean_T c1_x[2])
{
  boolean_T c1_y;
  int32_T c1_k;
  real_T c1_b_k;
  boolean_T c1_b1;
  boolean_T exitg1;
  c1_b_isequal(chartInstance);
  c1_y = false;
  c1_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c1_k < 2)) {
    c1_b_k = 1.0 + (real_T)c1_k;
    if ((real_T)c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
         ("", c1_b_k), 1, 2, 1, 0) - 1] == 0.0) {
      c1_b1 = true;
    } else {
      (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c1_b_k), 1, 2, 1, 0);
      c1_b1 = false;
    }

    if (!c1_b1) {
      c1_y = true;
      exitg1 = true;
    } else {
      c1_k++;
    }
  }

  return c1_y;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_nargout, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  boolean_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(boolean_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static boolean_T c1_c_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_sf_internal_predicateOutput, const char_T *c1_identifier)
{
  boolean_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_sf_internal_predicateOutput), &c1_thisId);
  sf_mex_destroy(&c1_sf_internal_predicateOutput);
  return c1_y;
}

static boolean_T c1_d_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  boolean_T c1_y;
  boolean_T c1_b2;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_b2, 1, 11, 0U, 0, 0U, 0);
  c1_y = c1_b2;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_sf_internal_predicateOutput;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  boolean_T c1_y;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_sf_internal_predicateOutput = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_sf_internal_predicateOutput), &c1_thisId);
  sf_mex_destroy(&c1_sf_internal_predicateOutput);
  *(boolean_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray
  *sf_c1_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 21, 1),
                false);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  const mxArray *c1_rhs1 = NULL;
  const mxArray *c1_lhs1 = NULL;
  const mxArray *c1_rhs2 = NULL;
  const mxArray *c1_lhs2 = NULL;
  const mxArray *c1_rhs3 = NULL;
  const mxArray *c1_lhs3 = NULL;
  const mxArray *c1_rhs4 = NULL;
  const mxArray *c1_lhs4 = NULL;
  const mxArray *c1_rhs5 = NULL;
  const mxArray *c1_lhs5 = NULL;
  const mxArray *c1_rhs6 = NULL;
  const mxArray *c1_lhs6 = NULL;
  const mxArray *c1_rhs7 = NULL;
  const mxArray *c1_lhs7 = NULL;
  const mxArray *c1_rhs8 = NULL;
  const mxArray *c1_lhs8 = NULL;
  const mxArray *c1_rhs9 = NULL;
  const mxArray *c1_lhs9 = NULL;
  const mxArray *c1_rhs10 = NULL;
  const mxArray *c1_lhs10 = NULL;
  const mxArray *c1_rhs11 = NULL;
  const mxArray *c1_lhs11 = NULL;
  const mxArray *c1_rhs12 = NULL;
  const mxArray *c1_lhs12 = NULL;
  const mxArray *c1_rhs13 = NULL;
  const mxArray *c1_lhs13 = NULL;
  const mxArray *c1_rhs14 = NULL;
  const mxArray *c1_lhs14 = NULL;
  const mxArray *c1_rhs15 = NULL;
  const mxArray *c1_lhs15 = NULL;
  const mxArray *c1_rhs16 = NULL;
  const mxArray *c1_lhs16 = NULL;
  const mxArray *c1_rhs17 = NULL;
  const mxArray *c1_lhs17 = NULL;
  const mxArray *c1_rhs18 = NULL;
  const mxArray *c1_lhs18 = NULL;
  const mxArray *c1_rhs19 = NULL;
  const mxArray *c1_lhs19 = NULL;
  const mxArray *c1_rhs20 = NULL;
  const mxArray *c1_lhs20 = NULL;
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c1_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c1_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c1_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c1_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c1_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c1_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("all"), "name", "name", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372589614U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c1_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c1_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c1_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372590358U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c1_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c1_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("isequal"), "name", "name", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c1_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c1_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c1_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("any"), "name", "name", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c1_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c1_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c1_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372590358U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c1_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("isnan"), "name", "name", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c1_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c1_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs20), "lhs", "lhs",
                  20);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
  sf_mex_destroy(&c1_rhs1);
  sf_mex_destroy(&c1_lhs1);
  sf_mex_destroy(&c1_rhs2);
  sf_mex_destroy(&c1_lhs2);
  sf_mex_destroy(&c1_rhs3);
  sf_mex_destroy(&c1_lhs3);
  sf_mex_destroy(&c1_rhs4);
  sf_mex_destroy(&c1_lhs4);
  sf_mex_destroy(&c1_rhs5);
  sf_mex_destroy(&c1_lhs5);
  sf_mex_destroy(&c1_rhs6);
  sf_mex_destroy(&c1_lhs6);
  sf_mex_destroy(&c1_rhs7);
  sf_mex_destroy(&c1_lhs7);
  sf_mex_destroy(&c1_rhs8);
  sf_mex_destroy(&c1_lhs8);
  sf_mex_destroy(&c1_rhs9);
  sf_mex_destroy(&c1_lhs9);
  sf_mex_destroy(&c1_rhs10);
  sf_mex_destroy(&c1_lhs10);
  sf_mex_destroy(&c1_rhs11);
  sf_mex_destroy(&c1_lhs11);
  sf_mex_destroy(&c1_rhs12);
  sf_mex_destroy(&c1_lhs12);
  sf_mex_destroy(&c1_rhs13);
  sf_mex_destroy(&c1_lhs13);
  sf_mex_destroy(&c1_rhs14);
  sf_mex_destroy(&c1_lhs14);
  sf_mex_destroy(&c1_rhs15);
  sf_mex_destroy(&c1_lhs15);
  sf_mex_destroy(&c1_rhs16);
  sf_mex_destroy(&c1_lhs16);
  sf_mex_destroy(&c1_rhs17);
  sf_mex_destroy(&c1_lhs17);
  sf_mex_destroy(&c1_rhs18);
  sf_mex_destroy(&c1_lhs18);
  sf_mex_destroy(&c1_rhs19);
  sf_mex_destroy(&c1_lhs19);
  sf_mex_destroy(&c1_rhs20);
  sf_mex_destroy(&c1_lhs20);
}

static const mxArray *c1_emlrt_marshallOut(const char * c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), false);
  return c1_y;
}

static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 7, 0U, 0U, 0U, 0), false);
  return c1_y;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_e_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i30;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i30, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i30;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_f_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_tp_Modes, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_tp_Modes),
    &c1_thisId);
  sf_mex_destroy(&c1_b_tp_Modes);
  return c1_y;
}

static uint8_T c1_g_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_tp_Modes;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_b_tp_Modes = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_tp_Modes),
    &c1_thisId);
  sf_mex_destroy(&c1_b_tp_Modes);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i31;
  real_T c1_b_inData[16];
  int32_T c1_i32;
  real_T c1_u[16];
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i31 = 0; c1_i31 < 16; c1_i31++) {
    c1_b_inData[c1_i31] = (*(real_T (*)[16])c1_inData)[c1_i31];
  }

  for (c1_i32 = 0; c1_i32 < 16; c1_i32++) {
    c1_u[c1_i32] = c1_b_inData[c1_i32];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 16), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i33;
  boolean_T c1_b_inData[2];
  int32_T c1_i34;
  boolean_T c1_u[2];
  const mxArray *c1_y = NULL;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i33 = 0; c1_i33 < 2; c1_i33++) {
    c1_b_inData[c1_i33] = (*(boolean_T (*)[2])c1_inData)[c1_i33];
  }

  for (c1_i34 = 0; c1_i34 < 2; c1_i34++) {
    c1_u[c1_i34] = c1_b_inData[c1_i34];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_h_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_frontSensors, const char_T *c1_identifier, boolean_T c1_y[2])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_frontSensors), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_b_frontSensors);
}

static void c1_i_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, boolean_T c1_y[2])
{
  boolean_T c1_bv10[2];
  int32_T c1_i35;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_bv10, 1, 11, 0U, 1, 0U, 1, 2);
  for (c1_i35 = 0; c1_i35 < 2; c1_i35++) {
    c1_y[c1_i35] = c1_bv10[c1_i35];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_frontSensors;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  boolean_T c1_y[2];
  int32_T c1_i36;
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
    chartInstanceVoid;
  c1_b_frontSensors = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_frontSensors), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_b_frontSensors);
  for (c1_i36 = 0; c1_i36 < 2; c1_i36++) {
    (*(boolean_T (*)[2])c1_outData)[c1_i36] = c1_y[c1_i36];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_j_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_b_setSimStateSideEffectsInfo, const char_T *c1_identifier)
{
  const mxArray *c1_y = NULL;
  emlrtMsgIdentifier c1_thisId;
  c1_y = NULL;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  sf_mex_assign(&c1_y, c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_setSimStateSideEffectsInfo), &c1_thisId), false);
  sf_mex_destroy(&c1_b_setSimStateSideEffectsInfo);
  return c1_y;
}

static const mxArray *c1_k_emlrt_marshallIn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  const mxArray *c1_y = NULL;
  (void)chartInstance;
  (void)c1_parentId;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_duplicatearraysafe(&c1_u), false);
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_updateDataWrittenToVector
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, uint32_T
   c1_vectorIndex)
{
  (void)chartInstance;
  c1_dataWrittenToVector[(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)
    c1_vectorIndex, 0, 6, 1, 0)] = true;
}

static void c1_errorIfDataNotWrittenToFcn
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance, uint32_T
   c1_vectorIndex, uint32_T c1_dataNumber)
{
  (void)chartInstance;
  _SFD_DATA_READ_BEFORE_WRITE_CHECK(c1_dataNumber, c1_dataWrittenToVector
    [(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)c1_vectorIndex, 0, 6, 1,
    0)]);
}

static void init_dsm_address_info
  (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance)
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

void sf_c1_Controller_Pioneer3DXAssembly_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(379646307U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(8959968U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4078356905U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2835668232U);
}

mxArray *sf_c1_Controller_Pioneer3DXAssembly_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("O0sURp2HYelOE2YAns7fwC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_Controller_Pioneer3DXAssembly_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c1_Controller_Pioneer3DXAssembly
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[34],T\"LeftVel\",},{M[1],M[35],T\"RightVel\",},{M[3],M[210],T\"frontSensors\",},{M[3],M[250],T\"leftMiddleSensors\",},{M[3],M[249],T\"leftMostSensors\",},{M[3],M[252],T\"rightMiddleSensors\",},{M[3],M[251],T\"rightMostSensors\",},{M[8],M[0],T\"is_active_c1_Controller_Pioneer3DXAssembly\",},{M[9],M[0],T\"is_c1_Controller_Pioneer3DXAssembly\",},{M[9],M[89],T\"is_Modes\",}}",
    "100 S1x2'type','srcId','name','auxInfo'{{M[11],M[0],T\"temporalCounter_i1\",S'et','os','ct'{{T\"at\",M[243],M[1]}}},{M[13],M[0],T\"previousTime\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 12, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_Controller_Pioneer3DXAssembly_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Controller_Pioneer3DXAssemblyMachineNumber_,
           1,
           10,
           15,
           0,
           10,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,"LeftVel");
          _SFD_SET_DATA_PROPS(1,2,0,1,"RightVel");
          _SFD_SET_DATA_PROPS(2,1,1,0,"SonarSensors");
          _SFD_SET_DATA_PROPS(3,0,0,0,"frontSensors");
          _SFD_SET_DATA_PROPS(4,1,1,0,"SensorRanges");
          _SFD_SET_DATA_PROPS(5,10,0,0,"v0");
          _SFD_SET_DATA_PROPS(6,0,0,0,"leftMostSensors");
          _SFD_SET_DATA_PROPS(7,0,0,0,"leftMiddleSensors");
          _SFD_SET_DATA_PROPS(8,0,0,0,"rightMostSensors");
          _SFD_SET_DATA_PROPS(9,0,0,0,"rightMiddleSensors");
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(1,0,0);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(5,0,0);
          _SFD_STATE_INFO(6,0,0);
          _SFD_STATE_INFO(7,0,0);
          _SFD_STATE_INFO(8,0,0);
          _SFD_STATE_INFO(9,0,0);
          _SFD_CH_SUBSTATE_COUNT(1);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_ST_SUBSTATE_COUNT(0,9);
          _SFD_ST_SUBSTATE_INDEX(0,0,1);
          _SFD_ST_SUBSTATE_INDEX(0,1,2);
          _SFD_ST_SUBSTATE_INDEX(0,2,3);
          _SFD_ST_SUBSTATE_INDEX(0,3,4);
          _SFD_ST_SUBSTATE_INDEX(0,4,5);
          _SFD_ST_SUBSTATE_INDEX(0,5,6);
          _SFD_ST_SUBSTATE_INDEX(0,6,7);
          _SFD_ST_SUBSTATE_INDEX(0,7,8);
          _SFD_ST_SUBSTATE_INDEX(0,8,9);
          _SFD_ST_SUBSTATE_COUNT(1,0);
          _SFD_ST_SUBSTATE_COUNT(2,0);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
          _SFD_ST_SUBSTATE_COUNT(5,0);
          _SFD_ST_SUBSTATE_COUNT(6,0);
          _SFD_ST_SUBSTATE_COUNT(7,0);
          _SFD_ST_SUBSTATE_COUNT(8,0);
          _SFD_ST_SUBSTATE_COUNT(9,0);
        }

        _SFD_CV_INIT_CHART(1,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,9,1,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(7,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(8,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(9,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(12,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(13,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(7,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(10,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(9,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(11,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(4,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(14,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(6,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(2,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(7,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(8,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(5,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(9,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(3,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(12,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(12,0,0,1,32,1,32);
        _SFD_CV_INIT_EML(13,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(8,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(8,0,0,1,23,1,23);
        _SFD_CV_INIT_EML(7,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(7,0,0,1,23,1,23);
        _SFD_CV_INIT_EML(10,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(10,0,0,1,25,1,25);
        _SFD_CV_INIT_EML(9,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(9,0,0,1,25,1,25);
        _SFD_CV_INIT_EML(2,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(2,0,0,1,20,1,20);
        _SFD_CV_INIT_EML(1,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(1,0,0,2,21,2,21);
        _SFD_CV_INIT_EML(11,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(11,0,0,1,20,1,20);
        _SFD_CV_INIT_EML(4,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(4,0,0,1,26,1,26);
        _SFD_CV_INIT_EML(3,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(3,0,0,1,26,1,26);
        _SFD_CV_INIT_EML(6,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(6,0,0,1,24,1,24);
        _SFD_CV_INIT_EML(5,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(5,0,0,1,24,1,24);
        _SFD_CV_INIT_EML(14,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(14,0,0,0,12,0,12);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_f_sf_marshallOut,(MexInFcnForType)
            c1_e_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)c1_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_f_sf_marshallOut,(MexInFcnForType)
            c1_e_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)c1_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_f_sf_marshallOut,(MexInFcnForType)
            c1_e_sf_marshallIn);
        }

        {
          real_T *c1_LeftVel;
          real_T *c1_RightVel;
          real_T (*c1_SonarSensors)[16];
          real_T (*c1_SensorRanges)[16];
          c1_SensorRanges = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 1);
          c1_SonarSensors = (real_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 0);
          c1_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_LeftVel);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_RightVel);
          _SFD_SET_DATA_VALUE_PTR(2U, *c1_SonarSensors);
          _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_frontSensors);
          _SFD_SET_DATA_VALUE_PTR(4U, *c1_SensorRanges);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c1_v0);
          _SFD_SET_DATA_VALUE_PTR(6U, &chartInstance->c1_leftMostSensors);
          _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c1_leftMiddleSensors);
          _SFD_SET_DATA_VALUE_PTR(8U, &chartInstance->c1_rightMostSensors);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c1_rightMiddleSensors);
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
  return "gMw571QwcQjDmysd8UdJxE";
}

static void sf_opaque_initialize_c1_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
  initialize_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  enable_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  disable_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  sf_gateway_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_Controller_Pioneer3DXAssembly
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_Controller_Pioneer3DXAssembly();/* state var info */
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

extern void sf_internal_set_sim_state_c1_Controller_Pioneer3DXAssembly(SimStruct*
  S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c1_Controller_Pioneer3DXAssembly();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_Controller_Pioneer3DXAssembly
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c1_Controller_Pioneer3DXAssembly(S);
}

static void sf_opaque_set_sim_state_c1_Controller_Pioneer3DXAssembly(SimStruct*
  S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_Controller_Pioneer3DXAssembly(S, st);
}

static void sf_opaque_terminate_c1_Controller_Pioneer3DXAssembly(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Controller_Pioneer3DXAssembly_optimization_info();
    }

    finalize_c1_Controller_Pioneer3DXAssembly
      ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_Controller_Pioneer3DXAssembly
    ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_Controller_Pioneer3DXAssembly(SimStruct *S)
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
    initialize_params_c1_Controller_Pioneer3DXAssembly
      ((SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  /* Actual parameters from chart:
     v0
   */
  const char_T *rtParamNames[] = { "v0" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for v0*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);
  ssSetNeedAbsoluteTime(S,1);
  ssSetModelReferenceSampleTimeDisallowInheritance(S);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Controller_Pioneer3DXAssembly_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2175855639U));
  ssSetChecksum1(S,(3095538040U));
  ssSetChecksum2(S,(959803069U));
  ssSetChecksum3(S,(1323262413U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c1_Controller_Pioneer3DXAssembly(SimStruct *S)
{
  SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct *)utMalloc
    (sizeof(SFc1_Controller_Pioneer3DXAssemblyInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc1_Controller_Pioneer3DXAssemblyInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_Controller_Pioneer3DXAssembly;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_Controller_Pioneer3DXAssembly;
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

void c1_Controller_Pioneer3DXAssembly_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_Controller_Pioneer3DXAssembly(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_Controller_Pioneer3DXAssembly(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_Controller_Pioneer3DXAssembly(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_Controller_Pioneer3DXAssembly_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
