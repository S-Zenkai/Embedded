/* S函数回代 */

#define S_FUNCTION_NAME  testPI
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include "PI.c"    //包含头文件


static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;   // 输入端口个数
    ssSetInputPortWidth(S, 0, 1);            // 输入端口维度
    ssSetInputPortRequiredContiguous(S, 0, true);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;  // 输出端口个数
    ssSetOutputPortWidth(S, 0, 1);           // 输出端口维度

    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, 0);
}
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.0001);   //设置采样周期时间
    ssSetOffsetTime(S, 0, 0.0);
}


PISTRU stru={0,0,0,0,0,0,0};         //设置初始状态

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T T_SampleTimes = ssGetSampleTime(S,0);
    real_T *x1 = ssGetRealDiscStates(S,0);

    const real_T *u0 = (const real_T*) ssGetInputPortSignal(S,0,1);

    real_T       *y1 = ssGetOutputPortSignal(S,0,1);

//========================调用代码主函数=========================//
   double l;
   l=u0[0];

   stru.Para.ki=1;
   stru.Para.kp=1;
   stru.Para.ts=0.0001;
   stru.Para.upLimit=70;
   stru.Para.dwLimit=-70;
   
   PIControl(l, &stru);

   y1[0]=stru.PI_out;

//=======================结束代码主函数=======================//

}

#define MDL_UPDATE
#if defined(MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid)
 {
  }
#endif


static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

