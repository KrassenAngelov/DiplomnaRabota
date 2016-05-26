/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RenesasCarOnTheTrack_sfun.h"
#include "c3_RenesasCarOnTheTrack.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RenesasCarOnTheTrack_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c3_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c3_IN_Modes                    ((uint8_T)1U)
#define c3_IN_GoStraight               ((uint8_T)1U)
#define c3_IN_TurnLeftTight            ((uint8_T)2U)
#define c3_IN_TurnLeftWide             ((uint8_T)3U)
#define c3_IN_TurnRightTight           ((uint8_T)4U)
#define c3_IN_TurnRightWide            ((uint8_T)5U)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c3_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_d_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_e_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_f_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_h_debug_family_names[2] = { "nargin", "nargout" };

static const char * c3_i_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_j_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_k_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_l_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_m_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_n_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_o_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_p_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_q_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_r_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_s_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_t_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_u_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_v_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_w_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c3_x_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static boolean_T c3_dataWrittenToVector[8];

/* Function Declarations */
static void initialize_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void initialize_params_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void enable_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void disable_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void set_sim_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray *c3_st);
static void c3_set_sim_state_side_effects_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void finalize_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void sf_gateway_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void initSimStructsc3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void c3_Modes(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static void c3_TurnRightWide(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance);
static boolean_T c3_all(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x);
static void c3_isequal(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static boolean_T c3_any(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x);
static boolean_T c3_b_all(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x[2]);
static void c3_b_isequal(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance);
static boolean_T c3_b_any(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x[2]);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_nargout, const char_T *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static boolean_T c3_c_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_sf_internal_predicateOutput, const char_T
  *c3_identifier);
static boolean_T c3_d_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(const mxArray **c3_info);
static const mxArray *c3_emlrt_marshallOut(const char * c3_u);
static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_e_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static uint8_T c3_f_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_b_tp_Modes, const char_T *c3_identifier);
static uint8_T c3_g_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_h_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_b_leftMiddleSensors, const char_T
  *c3_identifier, boolean_T c3_y[2]);
static void c3_i_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T c3_y[2]);
static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_j_emlrt_marshallIn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray
   *c3_b_setSimStateSideEffectsInfo, const char_T *c3_identifier);
static const mxArray *c3_k_emlrt_marshallIn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId);
static void c3_updateDataWrittenToVector(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, uint32_T c3_vectorIndex);
static void c3_errorIfDataNotWrittenToFcn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, uint32_T
   c3_vectorIndex, uint32_T c3_dataNumber);
static void init_dsm_address_info(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_doSetSimStateSideEffects = 0U;
  chartInstance->c3_setSimStateSideEffectsInfo = NULL;
  chartInstance->c3_is_Modes = c3_IN_NO_ACTIVE_CHILD;
  chartInstance->c3_tp_Modes = 0U;
  chartInstance->c3_tp_GoStraight = 0U;
  chartInstance->c3_tp_TurnLeftTight = 0U;
  chartInstance->c3_tp_TurnLeftWide = 0U;
  chartInstance->c3_tp_TurnRightTight = 0U;
  chartInstance->c3_tp_TurnRightWide = 0U;
  chartInstance->c3_is_active_c3_RenesasCarOnTheTrack = 0U;
  chartInstance->c3_is_c3_RenesasCarOnTheTrack = c3_IN_NO_ACTIVE_CHILD;
}

static void initialize_params_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  real_T c3_d0;
  sf_mex_import_named("v0", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      &c3_d0, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c3_v0 = c3_d0;
}

static void enable_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_update_debugger_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  uint32_T c3_prevAniVal;
  c3_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (chartInstance->c3_is_active_c3_RenesasCarOnTheTrack == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_c3_RenesasCarOnTheTrack == c3_IN_Modes) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_Modes == c3_IN_GoStraight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_Modes == c3_IN_TurnLeftTight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_Modes == c3_IN_TurnRightTight) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_Modes == c3_IN_TurnLeftWide) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c3_sfEvent);
  }

  if (chartInstance->c3_is_Modes == c3_IN_TurnRightWide) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c3_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c3_sfEvent);
  }

  _SFD_SET_ANIMATION(c3_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static const mxArray *get_sim_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  real_T c3_b_hoistedGlobal;
  real_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_c_hoistedGlobal;
  real_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  int32_T c3_i0;
  boolean_T c3_d_u[2];
  const mxArray *c3_e_y = NULL;
  boolean_T c3_d_hoistedGlobal;
  boolean_T c3_e_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_e_hoistedGlobal;
  real_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  int32_T c3_i1;
  boolean_T c3_g_u[2];
  const mxArray *c3_h_y = NULL;
  boolean_T c3_f_hoistedGlobal;
  boolean_T c3_h_u;
  const mxArray *c3_i_y = NULL;
  uint8_T c3_g_hoistedGlobal;
  uint8_T c3_i_u;
  const mxArray *c3_j_y = NULL;
  uint8_T c3_h_hoistedGlobal;
  uint8_T c3_j_u;
  const mxArray *c3_k_y = NULL;
  uint8_T c3_i_hoistedGlobal;
  uint8_T c3_k_u;
  const mxArray *c3_l_y = NULL;
  real_T *c3_LeftVel;
  real_T *c3_RightVel;
  c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(11, 1), false);
  c3_hoistedGlobal = *c3_LeftVel;
  c3_u = c3_hoistedGlobal;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = *c3_RightVel;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_c_hoistedGlobal = chartInstance->c3_frontSensors;
  c3_c_u = c3_c_hoistedGlobal;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 2, c3_d_y);
  for (c3_i0 = 0; c3_i0 < 2; c3_i0++) {
    c3_d_u[c3_i0] = chartInstance->c3_leftMiddleSensors[c3_i0];
  }

  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", c3_d_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_d_hoistedGlobal = chartInstance->c3_leftMostSensors;
  c3_e_u = c3_d_hoistedGlobal;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_e_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_e_hoistedGlobal = chartInstance->c3_lineLost;
  c3_f_u = c3_e_hoistedGlobal;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 5, c3_g_y);
  for (c3_i1 = 0; c3_i1 < 2; c3_i1++) {
    c3_g_u[c3_i1] = chartInstance->c3_rightMiddleSensors[c3_i1];
  }

  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", c3_g_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_setcell(c3_y, 6, c3_h_y);
  c3_f_hoistedGlobal = chartInstance->c3_rightMostSensors;
  c3_h_u = c3_f_hoistedGlobal;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_h_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 7, c3_i_y);
  c3_g_hoistedGlobal = chartInstance->c3_is_active_c3_RenesasCarOnTheTrack;
  c3_i_u = c3_g_hoistedGlobal;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 8, c3_j_y);
  c3_h_hoistedGlobal = chartInstance->c3_is_c3_RenesasCarOnTheTrack;
  c3_j_u = c3_h_hoistedGlobal;
  c3_k_y = NULL;
  sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_j_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 9, c3_k_y);
  c3_i_hoistedGlobal = chartInstance->c3_is_Modes;
  c3_k_u = c3_i_hoistedGlobal;
  c3_l_y = NULL;
  sf_mex_assign(&c3_l_y, sf_mex_create("y", &c3_k_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 10, c3_l_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  boolean_T c3_bv0[2];
  int32_T c3_i2;
  boolean_T c3_bv1[2];
  int32_T c3_i3;
  real_T *c3_LeftVel;
  real_T *c3_RightVel;
  c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_u = sf_mex_dup(c3_st);
  *c3_LeftVel = c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c3_u, 0)), "LeftVel");
  *c3_RightVel = c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c3_u, 1)), "RightVel");
  chartInstance->c3_frontSensors = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 2)), "frontSensors");
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 3)),
                        "leftMiddleSensors", c3_bv0);
  for (c3_i2 = 0; c3_i2 < 2; c3_i2++) {
    chartInstance->c3_leftMiddleSensors[c3_i2] = c3_bv0[c3_i2];
  }

  chartInstance->c3_leftMostSensors = c3_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c3_u, 4)), "leftMostSensors");
  chartInstance->c3_lineLost = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 5)), "lineLost");
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 6)),
                        "rightMiddleSensors", c3_bv1);
  for (c3_i3 = 0; c3_i3 < 2; c3_i3++) {
    chartInstance->c3_rightMiddleSensors[c3_i3] = c3_bv1[c3_i3];
  }

  chartInstance->c3_rightMostSensors = c3_c_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c3_u, 7)), "rightMostSensors");
  chartInstance->c3_is_active_c3_RenesasCarOnTheTrack = c3_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 8)),
     "is_active_c3_RenesasCarOnTheTrack");
  chartInstance->c3_is_c3_RenesasCarOnTheTrack = c3_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 9)),
     "is_c3_RenesasCarOnTheTrack");
  chartInstance->c3_is_Modes = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 10)), "is_Modes");
  sf_mex_assign(&chartInstance->c3_setSimStateSideEffectsInfo,
                c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c3_u, 11)), "setSimStateSideEffectsInfo"), true);
  sf_mex_destroy(&c3_u);
  chartInstance->c3_doSetSimStateSideEffects = 1U;
  c3_update_debugger_state_c3_RenesasCarOnTheTrack(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void c3_set_sim_state_side_effects_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  if (chartInstance->c3_doSetSimStateSideEffects != 0) {
    if (chartInstance->c3_is_c3_RenesasCarOnTheTrack == c3_IN_Modes) {
      chartInstance->c3_tp_Modes = 1U;
    } else {
      chartInstance->c3_tp_Modes = 0U;
    }

    if (chartInstance->c3_is_Modes == c3_IN_GoStraight) {
      chartInstance->c3_tp_GoStraight = 1U;
    } else {
      chartInstance->c3_tp_GoStraight = 0U;
    }

    if (chartInstance->c3_is_Modes == c3_IN_TurnLeftTight) {
      chartInstance->c3_tp_TurnLeftTight = 1U;
    } else {
      chartInstance->c3_tp_TurnLeftTight = 0U;
    }

    if (chartInstance->c3_is_Modes == c3_IN_TurnLeftWide) {
      chartInstance->c3_tp_TurnLeftWide = 1U;
    } else {
      chartInstance->c3_tp_TurnLeftWide = 0U;
    }

    if (chartInstance->c3_is_Modes == c3_IN_TurnRightTight) {
      chartInstance->c3_tp_TurnRightTight = 1U;
    } else {
      chartInstance->c3_tp_TurnRightTight = 0U;
    }

    if (chartInstance->c3_is_Modes == c3_IN_TurnRightWide) {
      chartInstance->c3_tp_TurnRightWide = 1U;
    } else {
      chartInstance->c3_tp_TurnRightWide = 0U;
    }

    chartInstance->c3_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  sf_mex_destroy(&chartInstance->c3_setSimStateSideEffectsInfo);
}

static void sf_gateway_c3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  int32_T c3_i4;
  int32_T c3_i5;
  int32_T c3_i6;
  int32_T c3_i7;
  uint32_T c3_debug_family_var_map[2];
  real_T c3_nargin = 0.0;
  real_T c3_nargout = 0.0;
  int32_T c3_i8;
  int32_T c3_i9;
  int32_T c3_i10;
  int32_T c3_i11;
  real_T c3_b_nargin = 0.0;
  real_T c3_b_nargout = 0.0;
  real_T *c3_LeftVel;
  real_T *c3_RightVel;
  real_T (*c3_SonarSensors)[8];
  real_T (*c3_SensorRanges)[8];
  c3_SensorRanges = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 1);
  c3_SonarSensors = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 0);
  c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_set_sim_state_side_effects_c3_RenesasCarOnTheTrack(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
  _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
  for (c3_i4 = 0; c3_i4 < 8; c3_i4++) {
    _SFD_DATA_RANGE_CHECK((*c3_SonarSensors)[c3_i4], 2U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c3_frontSensors, 3U);
  for (c3_i5 = 0; c3_i5 < 8; c3_i5++) {
    _SFD_DATA_RANGE_CHECK((*c3_SensorRanges)[c3_i5], 4U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c3_v0, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMostSensors, 6U);
  for (c3_i6 = 0; c3_i6 < 2; c3_i6++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMiddleSensors[c3_i6], 7U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMostSensors, 8U);
  for (c3_i7 = 0; c3_i7 < 2; c3_i7++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMiddleSensors[c3_i7],
                          9U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c3_lineLost, 10U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  if (chartInstance->c3_is_active_c3_RenesasCarOnTheTrack == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
    chartInstance->c3_is_active_c3_RenesasCarOnTheTrack = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
    chartInstance->c3_is_c3_RenesasCarOnTheTrack = c3_IN_Modes;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c3_sfEvent);
    chartInstance->c3_tp_Modes = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_debug_family_names,
      c3_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    chartInstance->c3_leftMostSensors = ((*c3_SonarSensors)[0] <
      (*c3_SensorRanges)[0]);
    c3_updateDataWrittenToVector(chartInstance, 3U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMostSensors, 6U);
    for (c3_i8 = 0; c3_i8 < 2; c3_i8++) {
      chartInstance->c3_leftMiddleSensors[c3_i8] = ((*c3_SonarSensors)[c3_i8 + 1]
        < (*c3_SensorRanges)[c3_i8 + 1]);
    }

    c3_updateDataWrittenToVector(chartInstance, 4U);
    for (c3_i9 = 0; c3_i9 < 2; c3_i9++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMiddleSensors[c3_i9],
                            7U);
    }

    for (c3_i10 = 0; c3_i10 < 2; c3_i10++) {
      chartInstance->c3_rightMiddleSensors[c3_i10] = ((*c3_SonarSensors)[c3_i10
        + 5] < (*c3_SensorRanges)[c3_i10 + 5]);
    }

    c3_updateDataWrittenToVector(chartInstance, 6U);
    for (c3_i11 = 0; c3_i11 < 2; c3_i11++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMiddleSensors[c3_i11],
                            9U);
    }

    chartInstance->c3_rightMostSensors = ((*c3_SonarSensors)[7] <
      (*c3_SensorRanges)[7]);
    c3_updateDataWrittenToVector(chartInstance, 5U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMostSensors, 8U);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, chartInstance->c3_sfEvent);
    chartInstance->c3_is_Modes = c3_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
    chartInstance->c3_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_c_debug_family_names,
      c3_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    *c3_RightVel = chartInstance->c3_v0;
    c3_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
    *c3_LeftVel = chartInstance->c3_v0;
    c3_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    c3_Modes(chartInstance);
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RenesasCarOnTheTrackMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc3_RenesasCarOnTheTrack
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c3_Modes(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  uint32_T c3_debug_family_var_map[2];
  real_T c3_nargin = 0.0;
  real_T c3_nargout = 0.0;
  int32_T c3_i12;
  int32_T c3_i13;
  int32_T c3_i14;
  int32_T c3_i15;
  uint32_T c3_b_debug_family_var_map[3];
  real_T c3_b_nargin = 0.0;
  real_T c3_b_nargout = 1.0;
  boolean_T c3_out;
  int32_T c3_i16;
  boolean_T c3_bv2[2];
  real_T c3_c_nargin = 0.0;
  real_T c3_c_nargout = 1.0;
  boolean_T c3_b_out;
  int32_T c3_i17;
  boolean_T c3_bv3[2];
  real_T c3_d_nargin = 0.0;
  real_T c3_d_nargout = 1.0;
  boolean_T c3_c_out;
  real_T c3_e_nargin = 0.0;
  real_T c3_e_nargout = 1.0;
  boolean_T c3_d_out;
  real_T c3_f_nargin = 0.0;
  real_T c3_f_nargout = 0.0;
  real_T c3_g_nargin = 0.0;
  real_T c3_g_nargout = 1.0;
  boolean_T c3_e_out;
  int32_T c3_i18;
  boolean_T c3_bv4[2];
  real_T c3_h_nargin = 0.0;
  real_T c3_h_nargout = 0.0;
  real_T c3_i_nargin = 0.0;
  real_T c3_i_nargout = 0.0;
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_A;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_y;
  real_T c3_j_nargin = 0.0;
  real_T c3_j_nargout = 1.0;
  boolean_T c3_f_out;
  real_T c3_k_nargin = 0.0;
  real_T c3_k_nargout = 0.0;
  real_T c3_l_nargin = 0.0;
  real_T c3_l_nargout = 0.0;
  real_T c3_c_hoistedGlobal;
  real_T c3_d_hoistedGlobal;
  real_T c3_b_A;
  real_T c3_d_x;
  real_T c3_e_x;
  real_T c3_f_x;
  real_T c3_b_y;
  real_T c3_m_nargin = 0.0;
  real_T c3_m_nargout = 1.0;
  boolean_T c3_g_out;
  int32_T c3_i19;
  boolean_T c3_bv5[2];
  real_T c3_n_nargin = 0.0;
  real_T c3_n_nargout = 0.0;
  real_T c3_o_nargin = 0.0;
  real_T c3_o_nargout = 0.0;
  real_T c3_e_hoistedGlobal;
  real_T c3_f_hoistedGlobal;
  real_T c3_c_A;
  real_T c3_g_x;
  real_T c3_h_x;
  real_T c3_i_x;
  real_T c3_c_y;
  real_T *c3_LeftVel;
  real_T *c3_RightVel;
  real_T (*c3_SonarSensors)[8];
  real_T (*c3_SensorRanges)[8];
  c3_SensorRanges = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 1);
  c3_SonarSensors = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S, 0);
  c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_b_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  chartInstance->c3_leftMostSensors = ((*c3_SonarSensors)[0] < (*c3_SensorRanges)
    [0]);
  c3_updateDataWrittenToVector(chartInstance, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMostSensors, 6U);
  for (c3_i12 = 0; c3_i12 < 2; c3_i12++) {
    chartInstance->c3_leftMiddleSensors[c3_i12] = ((*c3_SonarSensors)[c3_i12 + 1]
      < (*c3_SensorRanges)[c3_i12 + 1]);
  }

  c3_updateDataWrittenToVector(chartInstance, 4U);
  for (c3_i13 = 0; c3_i13 < 2; c3_i13++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_leftMiddleSensors[c3_i13],
                          7U);
  }

  for (c3_i14 = 0; c3_i14 < 2; c3_i14++) {
    chartInstance->c3_rightMiddleSensors[c3_i14] = ((*c3_SonarSensors)[c3_i14 +
      5] < (*c3_SensorRanges)[c3_i14 + 5]);
  }

  c3_updateDataWrittenToVector(chartInstance, 6U);
  for (c3_i15 = 0; c3_i15 < 2; c3_i15++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMiddleSensors[c3_i15],
                          9U);
  }

  chartInstance->c3_rightMostSensors = ((*c3_SonarSensors)[7] <
    (*c3_SensorRanges)[7]);
  c3_updateDataWrittenToVector(chartInstance, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c3_rightMostSensors, 8U);
  _SFD_SYMBOL_SCOPE_POP();
  switch (chartInstance->c3_is_Modes) {
   case c3_IN_GoStraight:
    CV_STATE_EVAL(0, 0, 1);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 7U, chartInstance->c3_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_l_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_out, 2U, c3_b_sf_marshallOut,
      c3_b_sf_marshallIn);
    c3_errorIfDataNotWrittenToFcn(chartInstance, 4U, 7U);
    for (c3_i16 = 0; c3_i16 < 2; c3_i16++) {
      c3_bv2[c3_i16] = ((real_T)chartInstance->c3_leftMiddleSensors[c3_i16] >
                        0.0);
    }

    c3_out = CV_EML_IF(7, 0, 0, c3_b_any(chartInstance, c3_bv2));
    _SFD_SYMBOL_SCOPE_POP();
    if (c3_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_GoStraight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
      chartInstance->c3_is_Modes = c3_IN_TurnLeftTight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_TurnLeftTight = 1U;
    } else {
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U,
                   chartInstance->c3_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_n_debug_family_names,
        c3_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_out, 2U, c3_b_sf_marshallOut,
        c3_b_sf_marshallIn);
      c3_errorIfDataNotWrittenToFcn(chartInstance, 6U, 9U);
      for (c3_i17 = 0; c3_i17 < 2; c3_i17++) {
        c3_bv3[c3_i17] = ((real_T)chartInstance->c3_rightMiddleSensors[c3_i17] >
                          0.0);
      }

      c3_b_out = CV_EML_IF(1, 0, 0, c3_b_any(chartInstance, c3_bv3));
      _SFD_SYMBOL_SCOPE_POP();
      if (c3_b_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
        chartInstance->c3_tp_GoStraight = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
        chartInstance->c3_is_Modes = c3_IN_TurnRightTight;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c3_sfEvent);
        chartInstance->c3_tp_TurnRightTight = 1U;
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 3U,
                     chartInstance->c3_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_p_debug_family_names,
          c3_b_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d_nargin, 0U, c3_sf_marshallOut,
          c3_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d_nargout, 1U,
          c3_sf_marshallOut, c3_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_out, 2U, c3_b_sf_marshallOut,
          c3_b_sf_marshallIn);
        c3_errorIfDataNotWrittenToFcn(chartInstance, 5U, 8U);
        c3_c_out = CV_EML_IF(3, 0, 0, c3_any(chartInstance, (real_T)
          chartInstance->c3_rightMostSensors > 0.0));
        _SFD_SYMBOL_SCOPE_POP();
        if (c3_c_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, chartInstance->c3_sfEvent);
          chartInstance->c3_tp_GoStraight = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
          chartInstance->c3_is_Modes = c3_IN_TurnRightWide;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c3_sfEvent);
          chartInstance->c3_tp_TurnRightWide = 1U;
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 5U,
                       chartInstance->c3_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_j_debug_family_names,
            c3_b_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_e_nargin, 0U,
            c3_sf_marshallOut, c3_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_e_nargout, 1U,
            c3_sf_marshallOut, c3_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d_out, 2U,
            c3_b_sf_marshallOut, c3_b_sf_marshallIn);
          c3_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
          c3_d_out = CV_EML_IF(5, 0, 0, c3_any(chartInstance, (real_T)
            chartInstance->c3_leftMostSensors > 0.0));
          _SFD_SYMBOL_SCOPE_POP();
          if (c3_d_out) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, chartInstance->c3_sfEvent);
            chartInstance->c3_tp_GoStraight = 0U;
            _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
            chartInstance->c3_is_Modes = c3_IN_TurnLeftWide;
            _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c3_sfEvent);
            chartInstance->c3_tp_TurnLeftWide = 1U;
          } else {
            _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 1U,
                         chartInstance->c3_sfEvent);
            _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_d_debug_family_names,
              c3_debug_family_var_map);
            _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f_nargin, 0U,
              c3_sf_marshallOut, c3_sf_marshallIn);
            _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f_nargout, 1U,
              c3_sf_marshallOut, c3_sf_marshallIn);
            *c3_RightVel = chartInstance->c3_v0;
            c3_updateDataWrittenToVector(chartInstance, 1U);
            _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
            *c3_LeftVel = chartInstance->c3_v0;
            c3_updateDataWrittenToVector(chartInstance, 0U);
            _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
            _SFD_SYMBOL_SCOPE_POP();
          }
        }
      }
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c3_sfEvent);
    break;

   case c3_IN_TurnLeftTight:
    CV_STATE_EVAL(0, 0, 2);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 8U, chartInstance->c3_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_k_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_g_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_g_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_e_out, 2U, c3_b_sf_marshallOut,
      c3_b_sf_marshallIn);
    c3_errorIfDataNotWrittenToFcn(chartInstance, 4U, 7U);
    for (c3_i18 = 0; c3_i18 < 2; c3_i18++) {
      c3_bv4[c3_i18] = ((real_T)chartInstance->c3_leftMiddleSensors[c3_i18] <
                        1.0);
    }

    c3_e_out = CV_EML_IF(8, 0, 0, c3_b_all(chartInstance, c3_bv4));
    _SFD_SYMBOL_SCOPE_POP();
    if (c3_e_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_TurnLeftTight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
      chartInstance->c3_is_Modes = c3_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_c_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_h_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_h_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      *c3_RightVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      *c3_LeftVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                   chartInstance->c3_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_e_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      *c3_RightVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      c3_hoistedGlobal = chartInstance->c3_v0;
      c3_b_hoistedGlobal = chartInstance->c3_v0;
      c3_A = c3_b_hoistedGlobal;
      c3_x = c3_A;
      c3_b_x = c3_x;
      c3_c_x = c3_b_x;
      c3_y = c3_c_x / 10.0;
      *c3_LeftVel = -c3_hoistedGlobal + c3_y;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
    break;

   case c3_IN_TurnLeftWide:
    CV_STATE_EVAL(0, 0, 3);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 6U, chartInstance->c3_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_i_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_j_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_j_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_f_out, 2U, c3_b_sf_marshallOut,
      c3_b_sf_marshallIn);
    c3_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
    c3_f_out = CV_EML_IF(6, 0, 0, c3_all(chartInstance, (real_T)
      chartInstance->c3_leftMostSensors < 1.0));
    _SFD_SYMBOL_SCOPE_POP();
    if (c3_f_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_TurnLeftWide = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c3_sfEvent);
      chartInstance->c3_is_Modes = c3_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_c_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      *c3_RightVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      *c3_LeftVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 3U,
                   chartInstance->c3_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_g_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_l_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_l_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      *c3_RightVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      c3_c_hoistedGlobal = chartInstance->c3_v0;
      c3_d_hoistedGlobal = chartInstance->c3_v0;
      c3_b_A = c3_d_hoistedGlobal;
      c3_d_x = c3_b_A;
      c3_e_x = c3_d_x;
      c3_f_x = c3_e_x;
      c3_b_y = c3_f_x / 3.0;
      *c3_LeftVel = -c3_c_hoistedGlobal + c3_b_y;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c3_sfEvent);
    break;

   case c3_IN_TurnRightTight:
    CV_STATE_EVAL(0, 0, 4);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 2U, chartInstance->c3_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_m_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_m_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_m_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_g_out, 2U, c3_b_sf_marshallOut,
      c3_b_sf_marshallIn);
    c3_errorIfDataNotWrittenToFcn(chartInstance, 6U, 9U);
    for (c3_i19 = 0; c3_i19 < 2; c3_i19++) {
      c3_bv5[c3_i19] = ((real_T)chartInstance->c3_rightMiddleSensors[c3_i19] <
                        1.0);
    }

    c3_g_out = CV_EML_IF(2, 0, 0, c3_b_all(chartInstance, c3_bv5));
    _SFD_SYMBOL_SCOPE_POP();
    if (c3_g_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_TurnRightTight = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c3_sfEvent);
      chartInstance->c3_is_Modes = c3_IN_GoStraight;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
      chartInstance->c3_tp_GoStraight = 1U;
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_c_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_n_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_n_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      *c3_RightVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      *c3_LeftVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                   chartInstance->c3_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_f_debug_family_names,
        c3_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_o_nargin, 0U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_o_nargout, 1U, c3_sf_marshallOut,
        c3_sf_marshallIn);
      c3_e_hoistedGlobal = chartInstance->c3_v0;
      c3_f_hoistedGlobal = chartInstance->c3_v0;
      c3_c_A = c3_f_hoistedGlobal;
      c3_g_x = c3_c_A;
      c3_h_x = c3_g_x;
      c3_i_x = c3_h_x;
      c3_c_y = c3_i_x / 10.0;
      *c3_RightVel = -c3_e_hoistedGlobal + c3_c_y;
      c3_updateDataWrittenToVector(chartInstance, 1U);
      _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
      *c3_LeftVel = chartInstance->c3_v0;
      c3_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }

    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c3_sfEvent);
    break;

   case c3_IN_TurnRightWide:
    CV_STATE_EVAL(0, 0, 5);
    c3_TurnRightWide(chartInstance);
    break;

   default:
    CV_STATE_EVAL(0, 0, 0);
    chartInstance->c3_is_Modes = c3_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
    break;
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
}

static void c3_TurnRightWide(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance)
{
  uint32_T c3_debug_family_var_map[3];
  real_T c3_nargin = 0.0;
  real_T c3_nargout = 1.0;
  boolean_T c3_out;
  uint32_T c3_b_debug_family_var_map[2];
  real_T c3_b_nargin = 0.0;
  real_T c3_b_nargout = 0.0;
  real_T c3_c_nargin = 0.0;
  real_T c3_c_nargout = 0.0;
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  real_T c3_A;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_y;
  real_T *c3_LeftVel;
  real_T *c3_RightVel;
  c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 4U, chartInstance->c3_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c3_o_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 1U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_out, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  c3_errorIfDataNotWrittenToFcn(chartInstance, 5U, 8U);
  c3_out = CV_EML_IF(4, 0, 0, c3_all(chartInstance, (real_T)
    chartInstance->c3_rightMostSensors < 1.0));
  _SFD_SYMBOL_SCOPE_POP();
  if (c3_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, chartInstance->c3_sfEvent);
    chartInstance->c3_tp_TurnRightWide = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c3_sfEvent);
    chartInstance->c3_is_Modes = c3_IN_GoStraight;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c3_sfEvent);
    chartInstance->c3_tp_GoStraight = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_c_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    *c3_RightVel = chartInstance->c3_v0;
    c3_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
    *c3_LeftVel = chartInstance->c3_v0;
    c3_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 5U, chartInstance->c3_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c3_h_debug_family_names,
      c3_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargin, 0U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_c_nargout, 1U, c3_sf_marshallOut,
      c3_sf_marshallIn);
    c3_hoistedGlobal = chartInstance->c3_v0;
    c3_b_hoistedGlobal = chartInstance->c3_v0;
    c3_A = c3_b_hoistedGlobal;
    c3_x = c3_A;
    c3_b_x = c3_x;
    c3_c_x = c3_b_x;
    c3_y = c3_c_x / 3.0;
    *c3_RightVel = -c3_hoistedGlobal + c3_y;
    c3_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK(*c3_RightVel, 1U);
    *c3_LeftVel = chartInstance->c3_v0;
    c3_updateDataWrittenToVector(chartInstance, 0U);
    _SFD_DATA_RANGE_CHECK(*c3_LeftVel, 0U);
    _SFD_SYMBOL_SCOPE_POP();
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c3_sfEvent);
}

static boolean_T c3_all(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x)
{
  boolean_T c3_b_x;
  c3_b_x = c3_x;
  c3_isequal(chartInstance);
  return (real_T)c3_b_x != 0.0;
}

static void c3_isequal(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c3_any(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x)
{
  boolean_T c3_b_x;
  boolean_T c3_b0;
  c3_b_x = c3_x;
  c3_isequal(chartInstance);
  c3_b0 = ((real_T)c3_b_x == 0.0);
  return !c3_b0;
}

static boolean_T c3_b_all(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x[2])
{
  boolean_T c3_y;
  int32_T c3_k;
  real_T c3_b_k;
  boolean_T exitg1;
  c3_b_isequal(chartInstance);
  c3_y = true;
  c3_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c3_k < 2)) {
    c3_b_k = 1.0 + (real_T)c3_k;
    if ((real_T)c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
         ("", c3_b_k), 1, 2, 1, 0) - 1] == 0.0) {
      c3_y = false;
      exitg1 = true;
    } else {
      c3_k++;
    }
  }

  return c3_y;
}

static void c3_b_isequal(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c3_b_any(SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance,
  boolean_T c3_x[2])
{
  boolean_T c3_y;
  int32_T c3_k;
  real_T c3_b_k;
  boolean_T c3_b1;
  boolean_T exitg1;
  c3_b_isequal(chartInstance);
  c3_y = false;
  c3_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c3_k < 2)) {
    c3_b_k = 1.0 + (real_T)c3_k;
    if ((real_T)c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
         ("", c3_b_k), 1, 2, 1, 0) - 1] == 0.0) {
      c3_b1 = true;
    } else {
      (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c3_b_k), 1, 2, 1, 0);
      c3_b1 = false;
    }

    if (!c3_b1) {
      c3_y = true;
      exitg1 = true;
    } else {
      c3_k++;
    }
  }

  return c3_y;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)c3_machineNumber;
  (void)c3_chartNumber;
  (void)c3_instanceNumber;
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_nargout, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  return c3_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d1;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d1, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d1;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  boolean_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(boolean_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static boolean_T c3_c_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_sf_internal_predicateOutput, const char_T
  *c3_identifier)
{
  boolean_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_sf_internal_predicateOutput), &c3_thisId);
  sf_mex_destroy(&c3_sf_internal_predicateOutput);
  return c3_y;
}

static boolean_T c3_d_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  boolean_T c3_y;
  boolean_T c3_b2;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b2, 1, 11, 0U, 0, 0U, 0);
  c3_y = c3_b2;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_sf_internal_predicateOutput;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  boolean_T c3_y;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_sf_internal_predicateOutput = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_sf_internal_predicateOutput), &c3_thisId);
  sf_mex_destroy(&c3_sf_internal_predicateOutput);
  *(boolean_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_RenesasCarOnTheTrack_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_createstruct("structure", 2, 21, 1),
                false);
  c3_info_helper(&c3_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(const mxArray **c3_info)
{
  const mxArray *c3_rhs0 = NULL;
  const mxArray *c3_lhs0 = NULL;
  const mxArray *c3_rhs1 = NULL;
  const mxArray *c3_lhs1 = NULL;
  const mxArray *c3_rhs2 = NULL;
  const mxArray *c3_lhs2 = NULL;
  const mxArray *c3_rhs3 = NULL;
  const mxArray *c3_lhs3 = NULL;
  const mxArray *c3_rhs4 = NULL;
  const mxArray *c3_lhs4 = NULL;
  const mxArray *c3_rhs5 = NULL;
  const mxArray *c3_lhs5 = NULL;
  const mxArray *c3_rhs6 = NULL;
  const mxArray *c3_lhs6 = NULL;
  const mxArray *c3_rhs7 = NULL;
  const mxArray *c3_lhs7 = NULL;
  const mxArray *c3_rhs8 = NULL;
  const mxArray *c3_lhs8 = NULL;
  const mxArray *c3_rhs9 = NULL;
  const mxArray *c3_lhs9 = NULL;
  const mxArray *c3_rhs10 = NULL;
  const mxArray *c3_lhs10 = NULL;
  const mxArray *c3_rhs11 = NULL;
  const mxArray *c3_lhs11 = NULL;
  const mxArray *c3_rhs12 = NULL;
  const mxArray *c3_lhs12 = NULL;
  const mxArray *c3_rhs13 = NULL;
  const mxArray *c3_lhs13 = NULL;
  const mxArray *c3_rhs14 = NULL;
  const mxArray *c3_lhs14 = NULL;
  const mxArray *c3_rhs15 = NULL;
  const mxArray *c3_lhs15 = NULL;
  const mxArray *c3_rhs16 = NULL;
  const mxArray *c3_lhs16 = NULL;
  const mxArray *c3_rhs17 = NULL;
  const mxArray *c3_lhs17 = NULL;
  const mxArray *c3_rhs18 = NULL;
  const mxArray *c3_lhs18 = NULL;
  const mxArray *c3_rhs19 = NULL;
  const mxArray *c3_lhs19 = NULL;
  const mxArray *c3_rhs20 = NULL;
  const mxArray *c3_lhs20 = NULL;
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c3_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c3_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c3_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c3_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c3_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c3_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c3_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("all"), "name", "name", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589614U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c3_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c3_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c3_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/all.m"), "context", "context",
                  10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590358U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c3_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c3_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isequal"), "name", "name", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825958U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c3_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1286825986U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c3_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c3_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(""), "context", "context", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("any"), "name", "name", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c3_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c3_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c3_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1372590358U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c3_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("isnan"), "name", "name", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c3_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c3_info, c3_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c3_info, c3_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c3_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c3_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c3_info, sf_mex_duplicatearraysafe(&c3_lhs20), "lhs", "lhs",
                  20);
  sf_mex_destroy(&c3_rhs0);
  sf_mex_destroy(&c3_lhs0);
  sf_mex_destroy(&c3_rhs1);
  sf_mex_destroy(&c3_lhs1);
  sf_mex_destroy(&c3_rhs2);
  sf_mex_destroy(&c3_lhs2);
  sf_mex_destroy(&c3_rhs3);
  sf_mex_destroy(&c3_lhs3);
  sf_mex_destroy(&c3_rhs4);
  sf_mex_destroy(&c3_lhs4);
  sf_mex_destroy(&c3_rhs5);
  sf_mex_destroy(&c3_lhs5);
  sf_mex_destroy(&c3_rhs6);
  sf_mex_destroy(&c3_lhs6);
  sf_mex_destroy(&c3_rhs7);
  sf_mex_destroy(&c3_lhs7);
  sf_mex_destroy(&c3_rhs8);
  sf_mex_destroy(&c3_lhs8);
  sf_mex_destroy(&c3_rhs9);
  sf_mex_destroy(&c3_lhs9);
  sf_mex_destroy(&c3_rhs10);
  sf_mex_destroy(&c3_lhs10);
  sf_mex_destroy(&c3_rhs11);
  sf_mex_destroy(&c3_lhs11);
  sf_mex_destroy(&c3_rhs12);
  sf_mex_destroy(&c3_lhs12);
  sf_mex_destroy(&c3_rhs13);
  sf_mex_destroy(&c3_lhs13);
  sf_mex_destroy(&c3_rhs14);
  sf_mex_destroy(&c3_lhs14);
  sf_mex_destroy(&c3_rhs15);
  sf_mex_destroy(&c3_lhs15);
  sf_mex_destroy(&c3_rhs16);
  sf_mex_destroy(&c3_lhs16);
  sf_mex_destroy(&c3_rhs17);
  sf_mex_destroy(&c3_lhs17);
  sf_mex_destroy(&c3_rhs18);
  sf_mex_destroy(&c3_lhs18);
  sf_mex_destroy(&c3_rhs19);
  sf_mex_destroy(&c3_lhs19);
  sf_mex_destroy(&c3_rhs20);
  sf_mex_destroy(&c3_lhs20);
}

static const mxArray *c3_emlrt_marshallOut(const char * c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c3_u)), false);
  return c3_y;
}

static const mxArray *c3_b_emlrt_marshallOut(const uint32_T c3_u)
{
  const mxArray *c3_y = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 7, 0U, 0U, 0U, 0), false);
  return c3_y;
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static int32_T c3_e_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i20;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i20, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i20;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  uint8_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(uint8_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static uint8_T c3_f_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_b_tp_Modes, const char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_tp_Modes),
    &c3_thisId);
  sf_mex_destroy(&c3_b_tp_Modes);
  return c3_y;
}

static uint8_T c3_g_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_tp_Modes;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  uint8_T c3_y;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_b_tp_Modes = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_tp_Modes),
    &c3_thisId);
  sf_mex_destroy(&c3_b_tp_Modes);
  *(uint8_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i21;
  real_T c3_b_inData[8];
  int32_T c3_i22;
  real_T c3_u[8];
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i21 = 0; c3_i21 < 8; c3_i21++) {
    c3_b_inData[c3_i21] = (*(real_T (*)[8])c3_inData)[c3_i21];
  }

  for (c3_i22 = 0; c3_i22 < 8; c3_i22++) {
    c3_u[c3_i22] = c3_b_inData[c3_i22];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i23;
  boolean_T c3_b_inData[2];
  int32_T c3_i24;
  boolean_T c3_u[2];
  const mxArray *c3_y = NULL;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i23 = 0; c3_i23 < 2; c3_i23++) {
    c3_b_inData[c3_i23] = (*(boolean_T (*)[2])c3_inData)[c3_i23];
  }

  for (c3_i24 = 0; c3_i24 < 2; c3_i24++) {
    c3_u[c3_i24] = c3_b_inData[c3_i24];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 11, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, false);
  return c3_mxArrayOutData;
}

static void c3_h_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_b_leftMiddleSensors, const char_T
  *c3_identifier, boolean_T c3_y[2])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_leftMiddleSensors),
                        &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_leftMiddleSensors);
}

static void c3_i_emlrt_marshallIn(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T c3_y[2])
{
  boolean_T c3_bv6[2];
  int32_T c3_i25;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_bv6, 1, 11, 0U, 1, 0U, 1, 2);
  for (c3_i25 = 0; c3_i25 < 2; c3_i25++) {
    c3_y[c3_i25] = c3_bv6[c3_i25];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_leftMiddleSensors;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  boolean_T c3_y[2];
  int32_T c3_i26;
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)chartInstanceVoid;
  c3_b_leftMiddleSensors = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_leftMiddleSensors),
                        &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_leftMiddleSensors);
  for (c3_i26 = 0; c3_i26 < 2; c3_i26++) {
    (*(boolean_T (*)[2])c3_outData)[c3_i26] = c3_y[c3_i26];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_j_emlrt_marshallIn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray
   *c3_b_setSimStateSideEffectsInfo, const char_T *c3_identifier)
{
  const mxArray *c3_y = NULL;
  emlrtMsgIdentifier c3_thisId;
  c3_y = NULL;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  sf_mex_assign(&c3_y, c3_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_setSimStateSideEffectsInfo), &c3_thisId), false);
  sf_mex_destroy(&c3_b_setSimStateSideEffectsInfo);
  return c3_y;
}

static const mxArray *c3_k_emlrt_marshallIn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, const mxArray *c3_u,
   const emlrtMsgIdentifier *c3_parentId)
{
  const mxArray *c3_y = NULL;
  (void)chartInstance;
  (void)c3_parentId;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_duplicatearraysafe(&c3_u), false);
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_updateDataWrittenToVector(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance, uint32_T c3_vectorIndex)
{
  (void)chartInstance;
  c3_dataWrittenToVector[(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)
    c3_vectorIndex, 0, 7, 1, 0)] = true;
}

static void c3_errorIfDataNotWrittenToFcn
  (SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance, uint32_T
   c3_vectorIndex, uint32_T c3_dataNumber)
{
  (void)chartInstance;
  _SFD_DATA_READ_BEFORE_WRITE_CHECK(c3_dataNumber, c3_dataWrittenToVector
    [(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)c3_vectorIndex, 0, 7, 1,
    0)]);
}

static void init_dsm_address_info(SFc3_RenesasCarOnTheTrackInstanceStruct
  *chartInstance)
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

void sf_c3_RenesasCarOnTheTrack_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1961860635U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2548746032U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3339343738U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2815253066U);
}

mxArray *sf_c3_RenesasCarOnTheTrack_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("PLNcMusHcg78jwnZcKvjQC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
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

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_RenesasCarOnTheTrack_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_RenesasCarOnTheTrack_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c3_RenesasCarOnTheTrack(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[34],T\"LeftVel\",},{M[1],M[35],T\"RightVel\",},{M[3],M[210],T\"frontSensors\",},{M[3],M[250],T\"leftMiddleSensors\",},{M[3],M[249],T\"leftMostSensors\",},{M[3],M[268],T\"lineLost\",},{M[3],M[252],T\"rightMiddleSensors\",},{M[3],M[251],T\"rightMostSensors\",},{M[8],M[0],T\"is_active_c3_RenesasCarOnTheTrack\",},{M[9],M[0],T\"is_c3_RenesasCarOnTheTrack\",}}",
    "100 S'type','srcId','name','auxInfo'{{M[9],M[89],T\"is_Modes\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 11, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_RenesasCarOnTheTrack_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RenesasCarOnTheTrackMachineNumber_,
           3,
           6,
           9,
           0,
           11,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_RenesasCarOnTheTrackMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_RenesasCarOnTheTrackMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _RenesasCarOnTheTrackMachineNumber_,
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
          _SFD_SET_DATA_PROPS(10,0,0,0,"lineLost");
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(1,0,0);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(5,0,0);
          _SFD_CH_SUBSTATE_COUNT(1);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_ST_SUBSTATE_COUNT(0,5);
          _SFD_ST_SUBSTATE_INDEX(0,0,1);
          _SFD_ST_SUBSTATE_INDEX(0,1,2);
          _SFD_ST_SUBSTATE_INDEX(0,2,3);
          _SFD_ST_SUBSTATE_INDEX(0,3,4);
          _SFD_ST_SUBSTATE_INDEX(0,4,5);
          _SFD_ST_SUBSTATE_COUNT(1,0);
          _SFD_ST_SUBSTATE_COUNT(2,0);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
          _SFD_ST_SUBSTATE_COUNT(5,0);
        }

        _SFD_CV_INIT_CHART(1,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,5,1,0,0,0,NULL,NULL);
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

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(7,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(4,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(2,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(3,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(5,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(6,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(6,0,0,1,23,1,23);
        _SFD_CV_INIT_EML(5,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(5,0,0,1,23,1,23);
        _SFD_CV_INIT_EML(8,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(8,0,0,1,25,1,25);
        _SFD_CV_INIT_EML(7,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(7,0,0,1,25,1,25);
        _SFD_CV_INIT_EML(2,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(2,0,0,1,26,1,26);
        _SFD_CV_INIT_EML(1,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(1,0,0,1,26,1,26);
        _SFD_CV_INIT_EML(4,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(4,0,0,1,24,1,24);
        _SFD_CV_INIT_EML(3,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(3,0,0,1,24,1,24);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)c3_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_f_sf_marshallOut,(MexInFcnForType)
            c3_e_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)c3_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_f_sf_marshallOut,(MexInFcnForType)
            c3_e_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);

        {
          real_T *c3_LeftVel;
          real_T *c3_RightVel;
          real_T (*c3_SonarSensors)[8];
          real_T (*c3_SensorRanges)[8];
          c3_SensorRanges = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S,
            1);
          c3_SonarSensors = (real_T (*)[8])ssGetInputPortSignal(chartInstance->S,
            0);
          c3_RightVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c3_LeftVel = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_LeftVel);
          _SFD_SET_DATA_VALUE_PTR(1U, c3_RightVel);
          _SFD_SET_DATA_VALUE_PTR(2U, *c3_SonarSensors);
          _SFD_SET_DATA_VALUE_PTR(3U, &chartInstance->c3_frontSensors);
          _SFD_SET_DATA_VALUE_PTR(4U, *c3_SensorRanges);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c3_v0);
          _SFD_SET_DATA_VALUE_PTR(6U, &chartInstance->c3_leftMostSensors);
          _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c3_leftMiddleSensors);
          _SFD_SET_DATA_VALUE_PTR(8U, &chartInstance->c3_rightMostSensors);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c3_rightMiddleSensors);
          _SFD_SET_DATA_VALUE_PTR(10U, &chartInstance->c3_lineLost);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _RenesasCarOnTheTrackMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "BViDxhZfAN2dFyh7oQVo8G";
}

static void sf_opaque_initialize_c3_RenesasCarOnTheTrack(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_RenesasCarOnTheTrack
    ((SFc3_RenesasCarOnTheTrackInstanceStruct*) chartInstanceVar);
  initialize_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_RenesasCarOnTheTrack(void *chartInstanceVar)
{
  enable_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_RenesasCarOnTheTrack(void *chartInstanceVar)
{
  disable_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_RenesasCarOnTheTrack(void *chartInstanceVar)
{
  sf_gateway_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_RenesasCarOnTheTrack
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_RenesasCarOnTheTrack
    ((SFc3_RenesasCarOnTheTrackInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_RenesasCarOnTheTrack();/* state var info */
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

extern void sf_internal_set_sim_state_c3_RenesasCarOnTheTrack(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c3_RenesasCarOnTheTrack();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_RenesasCarOnTheTrack(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c3_RenesasCarOnTheTrack(S);
}

static void sf_opaque_set_sim_state_c3_RenesasCarOnTheTrack(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c3_RenesasCarOnTheTrack(S, st);
}

static void sf_opaque_terminate_c3_RenesasCarOnTheTrack(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_RenesasCarOnTheTrackInstanceStruct*) chartInstanceVar
      )->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RenesasCarOnTheTrack_optimization_info();
    }

    finalize_c3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_RenesasCarOnTheTrack((SFc3_RenesasCarOnTheTrackInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_RenesasCarOnTheTrack(SimStruct *S)
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
    initialize_params_c3_RenesasCarOnTheTrack
      ((SFc3_RenesasCarOnTheTrackInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_RenesasCarOnTheTrack(SimStruct *S)
{
  /* Actual parameters from chart:
     v0
   */
  const char_T *rtParamNames[] = { "v0" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for v0*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RenesasCarOnTheTrack_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(87875522U));
  ssSetChecksum1(S,(3488660358U));
  ssSetChecksum2(S,(2381919093U));
  ssSetChecksum3(S,(2889785547U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_RenesasCarOnTheTrack(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c3_RenesasCarOnTheTrack(SimStruct *S)
{
  SFc3_RenesasCarOnTheTrackInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc3_RenesasCarOnTheTrackInstanceStruct *)utMalloc(sizeof
    (SFc3_RenesasCarOnTheTrackInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_RenesasCarOnTheTrackInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_RenesasCarOnTheTrack;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_RenesasCarOnTheTrack;
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

void c3_RenesasCarOnTheTrack_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_RenesasCarOnTheTrack(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_RenesasCarOnTheTrack(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_RenesasCarOnTheTrack(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_RenesasCarOnTheTrack_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
