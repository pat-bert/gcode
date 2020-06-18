/*
 * MATLAB Compiler: 8.0 (R2020a)
 * Date: Sun May 31 14:02:55 2020
 * Arguments:
 * "-B""macro_default""-W""lib:CollisionChecking,version=1.1""-T""link:lib""-d""
 */

#ifndef CollisionChecking_h
#define CollisionChecking_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_CollisionChecking_C_API 
#define LIB_CollisionChecking_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_CollisionChecking_C_API 
bool MW_CALL_CONV CollisionCheckingInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_CollisionChecking_C_API 
bool MW_CALL_CONV CollisionCheckingInitialize(void);

extern LIB_CollisionChecking_C_API 
void MW_CALL_CONV CollisionCheckingTerminate(void);

extern LIB_CollisionChecking_C_API 
void MW_CALL_CONV CollisionCheckingPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_CollisionChecking_C_API 
bool MW_CALL_CONV mlxValidate_config(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                     *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

/* C INTERFACE -- MLF WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_CollisionChecking_C_API bool MW_CALL_CONV mlfValidate_config(int nargout, mxArray** isCollision, mxArray** selfCollisionPairIdx, mxArray** worldCollisionPairIdx, mxArray* current_config, mxArray* interactive, mxArray* urdf_path);

#ifdef __cplusplus
}
#endif
/* C INTERFACE -- MLF WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#endif
