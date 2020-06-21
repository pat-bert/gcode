/*
 * MATLAB Compiler: 8.0 (R2020a)
 * Date: Sun May 31 14:02:55 2020
 * Arguments:
 * "-B""macro_default""-W""lib:CollisionChecking,version=1.1""-T""link:lib""-d""
 */

#define EXPORTING_CollisionChecking 1
#include "CollisionChecking.h"

static HMCRINSTANCE _mcr_inst = NULL; /* don't use nullptr; this may be either C or C++ */

#if defined( _MSC_VER) || defined(__LCC__) || defined(__MINGW64__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define NOMINMAX
#include <windows.h>
#undef interface

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern C block */
#endif

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern C block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_CollisionChecking_C_API
#define LIB_CollisionChecking_C_API /* No special import/export declaration */
#endif

LIB_CollisionChecking_C_API 
bool MW_CALL_CONV CollisionCheckingInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst)
        return true;
    if (!mclmcrInitialize())
        return false;
    if (!GetModuleFileName(GetModuleHandle("CollisionChecking"), path_to_dll, _MAX_PATH))
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream(path_to_dll);
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_CollisionChecking_C_API 
bool MW_CALL_CONV CollisionCheckingInitialize(void)
{
    return CollisionCheckingInitializeWithHandlers(mclDefaultErrorHandler, 
                                                 mclDefaultPrintHandler);
}

LIB_CollisionChecking_C_API 
void MW_CALL_CONV CollisionCheckingTerminate(void)
{
    if (_mcr_inst)
        mclTerminateInstance(&_mcr_inst);
}

LIB_CollisionChecking_C_API 
void MW_CALL_CONV CollisionCheckingPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_CollisionChecking_C_API 
bool MW_CALL_CONV mlxValidate_config(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "validate_config", nlhs, plhs, nrhs, prhs);
}

LIB_CollisionChecking_C_API 
bool MW_CALL_CONV mlfValidate_config(int nargout, mxArray** isCollision, mxArray** 
                                     selfCollisionPairIdx, mxArray** 
                                     worldCollisionPairIdx, mxArray* current_config, 
                                     mxArray* interactive, mxArray* urdf_path)
{
    return mclMlfFeval(_mcr_inst, "validate_config", nargout, 3, 3, isCollision, selfCollisionPairIdx, worldCollisionPairIdx, current_config, interactive, urdf_path);
}

