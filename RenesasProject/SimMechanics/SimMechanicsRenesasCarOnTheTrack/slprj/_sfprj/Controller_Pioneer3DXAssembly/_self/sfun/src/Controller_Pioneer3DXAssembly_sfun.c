/* Include files */

#include "Controller_Pioneer3DXAssembly_sfun.h"
#include "Controller_Pioneer3DXAssembly_sfun_debug_macros.h"
#include "c1_Controller_Pioneer3DXAssembly.h"
#include "c2_Controller_Pioneer3DXAssembly.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _Controller_Pioneer3DXAssemblyMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void Controller_Pioneer3DXAssembly_initializer(void)
{
}

void Controller_Pioneer3DXAssembly_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_Controller_Pioneer3DXAssembly_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_Controller_Pioneer3DXAssembly_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_Controller_Pioneer3DXAssembly_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  return 0;
}

unsigned int sf_Controller_Pioneer3DXAssembly_process_check_sum_call( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(84383263U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(614454322U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1931311539U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(842441594U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2287025347U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2213824637U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3447881020U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4283747939U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_Controller_Pioneer3DXAssembly_get_check_sum(mxArray *
            plhs[]);
          sf_c1_Controller_Pioneer3DXAssembly_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_Controller_Pioneer3DXAssembly_get_check_sum(mxArray *
            plhs[]);
          sf_c2_Controller_Pioneer3DXAssembly_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3031367619U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4001028638U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3978939492U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(838979348U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(90640147U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(505920300U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1218368360U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(421548441U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Controller_Pioneer3DXAssembly_autoinheritance_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "O0sURp2HYelOE2YAns7fwC") == 0) {
          extern mxArray
            *sf_c1_Controller_Pioneer3DXAssembly_get_autoinheritance_info(void);
          plhs[0] = sf_c1_Controller_Pioneer3DXAssembly_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "nfuwC8ndxMDajDwVxDy9SH") == 0) {
          extern mxArray
            *sf_c2_Controller_Pioneer3DXAssembly_get_autoinheritance_info(void);
          plhs[0] = sf_c2_Controller_Pioneer3DXAssembly_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_Controller_Pioneer3DXAssembly_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_Controller_Pioneer3DXAssembly_third_party_uses_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "gMw571QwcQjDmysd8UdJxE") == 0) {
          extern mxArray
            *sf_c1_Controller_Pioneer3DXAssembly_third_party_uses_info(void);
          plhs[0] = sf_c1_Controller_Pioneer3DXAssembly_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "xU27y96WtTY23iucuDiKSC") == 0) {
          extern mxArray
            *sf_c2_Controller_Pioneer3DXAssembly_third_party_uses_info(void);
          plhs[0] = sf_c2_Controller_Pioneer3DXAssembly_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "gMw571QwcQjDmysd8UdJxE") == 0) {
          extern mxArray
            *sf_c1_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info(void);
          plhs[0] =
            sf_c1_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "xU27y96WtTY23iucuDiKSC") == 0) {
          extern mxArray
            *sf_c2_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info(void);
          plhs[0] =
            sf_c2_Controller_Pioneer3DXAssembly_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void Controller_Pioneer3DXAssembly_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _Controller_Pioneer3DXAssemblyMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"Controller_Pioneer3DXAssembly","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _Controller_Pioneer3DXAssemblyMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _Controller_Pioneer3DXAssemblyMachineNumber_,0);
}

void Controller_Pioneer3DXAssembly_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_Controller_Pioneer3DXAssembly_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "Controller_Pioneer3DXAssembly", "Controller_Pioneer3DXAssembly");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_Controller_Pioneer3DXAssembly_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
