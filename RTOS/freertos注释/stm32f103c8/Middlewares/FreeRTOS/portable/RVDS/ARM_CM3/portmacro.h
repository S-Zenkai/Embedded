/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#ifndef PORTMACRO_H
#define PORTMACRO_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
    #define portCHAR          char
    #define portFLOAT         float
    #define portDOUBLE        double
    #define portLONG          long
    #define portSHORT         short
    #define portSTACK_TYPE    uint32_t
    #define portBASE_TYPE     long

    typedef portSTACK_TYPE   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;

    #if ( configUSE_16_BIT_TICKS == 1 )
        typedef uint16_t     TickType_t;
        #define portMAX_DELAY              ( TickType_t ) 0xffff
    #else
        typedef uint32_t     TickType_t;
        #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
        #define portTICK_TYPE_IS_ATOMIC    1
    #endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
    #define portSTACK_GROWTH          ( -1 )
    #define portTICK_PERIOD_MS        ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
    #define portBYTE_ALIGNMENT        8

/* Constants used with memory barrier intrinsics. */
    #define portSY_FULL_READ_WRITE    ( 15 )

/*-----------------------------------------------------------*/

/* Scheduler utilities. */
    #define portYIELD()                                 \
    {                                                   \
        /* Set a PendSV to request a context switch. */ \
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; \
                                                        \
        /* Barriers are normally not required but do ensure the code is completely \
         * within the specified behaviour for the architecture. */ \
        __dsb( portSY_FULL_READ_WRITE );                           \
        __isb( portSY_FULL_READ_WRITE );                           \
    }
/*-----------------------------------------------------------*/

    #define portNVIC_INT_CTRL_REG     ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
    #define portNVIC_PENDSVSET_BIT    ( 1UL << 28UL )
    #define portEND_SWITCHING_ISR( xSwitchRequired )    do { if( xSwitchRequired != pdFALSE ) portYIELD(); } while( 0 )
    #define portYIELD_FROM_ISR( x )                     portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Critical section management. */
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );

    #define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()
    #define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )
    #define portENTER_CRITICAL()                      vPortEnterCritical()
    #define portEXIT_CRITICAL()                       vPortExitCritical()
    #define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortRaiseBASEPRI()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortSetBASEPRI( x )

/*-----------------------------------------------------------*/

/* Tickless idle/low power functionality. */
    #ifndef portSUPPRESS_TICKS_AND_SLEEP
        extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
        #define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime )    vPortSuppressTicksAndSleep( xExpectedIdleTime )
    #endif
/*-----------------------------------------------------------*/

/* Port specific optimisations. */
    #ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
        #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
    #endif

    #if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

/* Check the configuration. */
        #if ( configMAX_PRIORITIES > 32 )
            #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
        #endif

/* Store/clear the ready priorities in a bit map. */
/*uxReadyPriorities:就绪优先级位图，调度器通过它可以快速找到最高优先级的就绪任务，如果某一优先级的任务跑完了，则清楚对应的位*/
        #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
        #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

        #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    uxTopPriority = ( 31UL - ( uint32_t ) __clz( ( uxReadyPriorities ) ) )

    #endif /* taskRECORD_READY_PRIORITY */
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
 * not necessary for to use this port.  They are defined so the common demo files
 * (which build with all the ports) will build. */
    #define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
    #define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )
/*-----------------------------------------------------------*/

    #ifdef configASSERT
        void vPortValidateInterruptPriority( void );
        #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
    #endif

/* portNOP() is not required by this port. */
    #define portNOP()

    #define portINLINE              __inline

    #ifndef portFORCE_INLINE
        #define portFORCE_INLINE    __forceinline
    #endif

/*-----------------------------------------------------------*/
		/*开中断，将basepri寄存器置0*/
    static portFORCE_INLINE void vPortSetBASEPRI( uint32_t ulBASEPRI )
    {
        __asm
        {
            /* Barrier instructions are not used as this function is only used to
             * lower the BASEPRI value. */
/* *INDENT-OFF* */
            msr basepri, ulBASEPRI
/* *INDENT-ON* */
        }
    }
/*-----------------------------------------------------------*/
    /*关中断*/
    static portFORCE_INLINE void vPortRaiseBASEPRI( void )
    {
        /*basepri寄存器屏蔽优先级低于某一阈值的中断*/
        uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

        __asm
        {
            /* Set BASEPRI to the max syscall priority to effect a critical
             * section. */
/* *INDENT-OFF* */
            msr basepri, ulNewBASEPRI
            dsb
            isb
/* *INDENT-ON* */
        }
    }
/*-----------------------------------------------------------*/

    static portFORCE_INLINE void vPortClearBASEPRIFromISR( void )
    {
        __asm
        {
            /* Set BASEPRI to 0 so no interrupts are masked.  This function is only
             * used to lower the mask in an interrupt, so memory barriers are not
             * used. */
/* *INDENT-OFF* */
            msr basepri, # 0
/* *INDENT-ON* */
        }
    }
/*-----------------------------------------------------------*/

    static portFORCE_INLINE uint32_t ulPortRaiseBASEPRI( void )
    {
        uint32_t ulReturn, ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

        __asm
        {
            /* Set BASEPRI to the max syscall priority to effect a critical
             * section. */
/* *INDENT-OFF* */
            mrs ulReturn, basepri
            msr basepri, ulNewBASEPRI
            dsb
            isb
/* *INDENT-ON* */
        }

        return ulReturn;
    }
/*-----------------------------------------------------------*/

    static portFORCE_INLINE BaseType_t xPortIsInsideInterrupt( void )
    {
        uint32_t ulCurrentInterrupt;
        BaseType_t xReturn;

        /* Obtain the number of the currently executing interrupt. */
        __asm
        {
/* *INDENT-OFF* */
            mrs ulCurrentInterrupt, ipsr
/* *INDENT-ON* */
        }

        if( ulCurrentInterrupt == 0 )
        {
            xReturn = pdFALSE;
        }
        else
        {
            xReturn = pdTRUE;
        }

        return xReturn;
    }


/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
