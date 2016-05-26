#ifndef __c1_Controller_Pioneer3DXAssembly_h__
#define __c1_Controller_Pioneer3DXAssembly_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_Controller_Pioneer3DXAssemblyInstanceStruct
#define typedef_SFc1_Controller_Pioneer3DXAssemblyInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  uint8_T c1_tp_Modes;
  uint8_T c1_tp_TurnLeftWide;
  uint8_T c1_tp_GoStraight;
  uint8_T c1_tp_DetermineLeftRight;
  uint8_T c1_tp_TurnRightInPlace;
  uint8_T c1_tp_TurnLeftInPlace;
  uint8_T c1_tp_TurnRightTight;
  uint8_T c1_tp_TurnLeftTight;
  uint8_T c1_tp_TurnRightWide;
  uint8_T c1_tp_ReverseABit;
  boolean_T c1_isStable;
  uint8_T c1_is_active_c1_Controller_Pioneer3DXAssembly;
  uint8_T c1_is_c1_Controller_Pioneer3DXAssembly;
  uint8_T c1_is_Modes;
  boolean_T c1_frontSensors[2];
  real_T c1_v0;
  boolean_T c1_leftMostSensors;
  boolean_T c1_leftMiddleSensors[2];
  boolean_T c1_rightMostSensors;
  boolean_T c1_rightMiddleSensors[2];
  real_T c1_temporalCounter_i1;
  real_T c1_presentTime;
  real_T c1_elapsedTime;
  real_T c1_previousTime;
  uint8_T c1_doSetSimStateSideEffects;
  const mxArray *c1_setSimStateSideEffectsInfo;
} SFc1_Controller_Pioneer3DXAssemblyInstanceStruct;

#endif                                 /*typedef_SFc1_Controller_Pioneer3DXAssemblyInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_Controller_Pioneer3DXAssembly_get_check_sum(mxArray *plhs[]);
extern void c1_Controller_Pioneer3DXAssembly_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
