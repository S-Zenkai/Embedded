/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 * Copyright 1990-2018 The MathWorks, Inc.
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  sfuntmpl_basic
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"



/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* 设置 S-Function 模块期望的参数数量。这些参数是用户可以在 Simulink 模块对话框中输入的配置值 */
    /*比较设定参数数量与实际参数是否相等*/
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
    /*状态设置*/
    /*设置模块内部状态的数量。状态是模块的“记忆”，使得输出不仅依赖于当前输入，还依赖于过去的历史。*/
    ssSetNumContStates(S, 0);/*设置连续状态的数量*/
    ssSetNumDiscStates(S, 0);/*设置离散状态的数量*/
    
    /*设置模块的输入端口数量。，设置失败会返回*/
    if (!ssSetNumInputPorts(S, 1)) return;
    /*设置指定输入端口的信号宽度（维度）。*/
    /*参数：(S, 端口索引, 宽度)。端口索引从 0 开始。*/
    /*(S, 0, 1) 表示第 0 个（也就是唯一的）输入端口接收一个宽度为 1 的信号，即一个标量*/
    ssSetInputPortWidth(S, 0, 1);
    /*性能优化设置。它要求 Simulink 为该输入端口的信号分配一块连续的内存空间。*/
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     */
    /*功能: 这是 S-Function 中最关键的设置之一。它声明了输入信号是否直接用于计算输出。
     *Direct Feedthrough (直接馈通): 如果一个模块的输出值在当前时刻 t 依赖于其输入
     *值在同一时刻 t 的值，那么这个模块就具有直接馈通特性。
     *参数: 1 (或 true) 表示有直接馈通。0 (或 false) 表示没有。
     *本例中: 1 表示这个模块的输出 y(t) 会用到输入 u(t)。例如 y(t) = 2 * u(t)。
     *如果是一个积分器 y(t) = ∫u(t)dt，它的输出在 t 时刻仅依赖于过去的状态，
     *不直接依赖 u(t)，则其直接馈通为 0。*/
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    /*设置模块的输出端口数量。设置失败会返回*/
    if (!ssSetNumOutputPorts(S, 1)) return;
    /*设置指定输出端口的信号宽度（维度）。*/
    /*参数：(S, 端口索引, 宽度)。端口索引从 0 开始。*/
    /*(S, 0, 1) 表示第 0 个（也就是唯一的）输出端口输出一个宽度为 1 的信号，即一个标量*/
    ssSetOutputPortWidth(S, 0, 1);
    
    /*设置模块具有的采样时间数量*/
    ssSetNumSampleTimes(S, 1);
    /*功能: 设置各种“工作向量”和高级功能的数量。这些是 S-Function 用来存储内部数据的预留内存空间。
     *RWork: 实数工作向量 (double)。
     *IWork: 整数工作向量 (int)。
     *PWork: 指针工作向量 (void*)。
     *Modes: 模式向量，用于具有不同操作模式的复杂模块。
     *NonsampledZCs: 非采样过零检测点，用于事件检测。
     */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the operating point save/restore compliance to be same as a 
     * built-in block */
    /*功能: 定义模块如何处理工作点 (Operating Point) 的保存和恢复。
     *工作点是仿真某一时刻所有状态的快照，用于线性化分析或从特定点继续仿真。
     *本例中: USE_DEFAULT_OPERATING_POINT 表示让 Simulink 采用和内置模块相同的默认行为来处理。
     *由于此模块无状态，这个设置影响很小。
     */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    
    /*功能: 声明 S-Function 的代码是线程安全的。
     *意义: 如果声明为 TRUE，Simulink 可以在多核处理器上并行执行此模块的多个实例
     *（例如在 "For Each Subsystem" 中），从而可能加速仿真。
     *这要求代码中不能使用会引起线程冲突的全局变量或静态变量。对于这个简单的无状态模块，
     *声明为线程安全是合适的。
     */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    /*这是一个性能优化选项*/
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



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
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *u = (const real_T*) ssGetInputPortSignal(S,0);
    real_T       *y = ssGetOutputPortSignal(S,0);
    y[0] = u[0];
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
