/* S�����ش� */

#define S_FUNCTION_NAME  test1
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include "atti_control.h"   //����ͷ�ļ�
#include "com_data.h"


static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;   // ����˿ڸ���
    ssSetInputPortWidth(S, 0, 1);            // ����˿�ά��
    ssSetInputPortRequiredContiguous(S, 0, true);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetInputPortWidth(S, 1, 1);            // ����˿�ά��
    ssSetInputPortRequiredContiguous(S, 1, true);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;  // ����˿ڸ���
    ssSetOutputPortWidth(S, 0, 1);           // ����˿�ά��

    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, 0);
    ssPrintf("��ʼ��ģ��\n");
}
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.0001);   //���ò�������ʱ��
    ssSetOffsetTime(S, 0, 0.0);
    ssPrintf("����ʱ��������\n");
}


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
      const real_T *u0 = (const real_T*) ssGetInputPortSignal(S,0);
      flight_control_state.control_state[YAW].rate=u0[0];
      atti_control_init();
      
      ssPrintf("atti_control_ctx.ang_vel_pid[YAW].Kp=%f\n",atti_control_ctx.ang_vel_pid[YAW].Kp);
  }
#endif /*  MDL_START */


// PISTRU stru={0,0,0,0,0,0,0};         //���ó�ʼ״̬

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T T_SampleTimes = ssGetSampleTime(S,0);
    // real_T *x1 = ssGetRealDiscStates(S,0);/*����������*/
    
    const real_T *u0 = (const real_T*) ssGetInputPortSignal(S,0);
    const real_T *u1 = (const real_T*) ssGetInputPortSignal(S,1);
    real_T       *y1 = ssGetOutputPortSignal(S,0);
    
//========================���ô���������=========================//
   double l;
   l=u0[0];
    
   atti_control_ctx.rc_state->norm.rc_yaw=u0[0];
   // ssPrintf("atti_control_ctx.rc_state->norm.rc_yaw=%f\n",atti_control_ctx.rc_state->norm.rc_yaw);
   atti_control_ctx.flight_control_state->control_state[YAW].rate=u1[0];
   atti_control_ctx.flight_control_state->throttle = 500.0;
   atti_control_update();
   y1[0] = atti_control_ctx.ang_vel_pid[YAW].output;
   // stru.Para.ki=1;
   // stru.Para.kp=1;
   // stru.Para.ts=0.0001;
   // stru.Para.upLimit=70;
   // stru.Para.dwLimit=-70;
   
   // PIControl(l, &stru);

//    y1[0]=stru.PI_out;

//=======================��������������=======================//

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

