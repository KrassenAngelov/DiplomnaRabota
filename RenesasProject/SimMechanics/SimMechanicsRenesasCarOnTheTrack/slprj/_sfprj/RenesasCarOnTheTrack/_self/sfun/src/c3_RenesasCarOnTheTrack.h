#ifndef __c3_RenesasCarOnTheTrack_h__
#define __c3_RenesasCarOnTheTrack_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_RenesasCarOnTheTrackInstanceStruct
#define typedef_SFc3_RenesasCarOnTheTrackInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  uint8_T c3_tp_Modes;
  uint8_T c3_tp_GoStraight;
  uint8_T c3_tp_TurnLeftTight;
  uint8_T c3_tp_TurnRightTight;
  uint8_T c3_tp_TurnLeftWide;
  uint8_T c3_tp_TurnRightWide;
  boolean_T c3_isStable;
  uint8_T c3_is_active_c3_RenesasCarOnTheTrack;
  uint8_T c3_is_c3_RenesasCarOnTheTrack;
  uint8_T c3_is_Modes;
  real_T c3_frontSensors;
  real_T c3_v0;
  boolean_T c3_leftMostSensors;
  boolean_T c3_leftMiddleSensors[2];
  boolean_T c3_rightMostSensors;
  boolean_T c3_rightMiddleSensors[2];
  real_T c3_lineLost;
  uint8_T c3_doSetSimStateSideEffects;
  const mxArray *c3_setSimStateSideEffectsInfo;
} SFc3_RenesasCarOnTheTrackInstanceStruct;

#endif                                 /*typedef_SFc3_RenesasCarOnTheTrackInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_RenesasCarOnTheTrack_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c3_RenesasCarOnTheTrack_get_check_sum(mxArray *plhs[]);
extern void c3_RenesasCarOnTheTrack_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
