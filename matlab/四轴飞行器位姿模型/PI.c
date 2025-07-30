/*
 * PI.c
 *
 *  Created on: 2023��1��8��
 *      Author: Administrator
 */
#include <math.h>

typedef struct{
    float kp;
    float ki;
    float upLimit;
    float dwLimit;
    float ts;
}PIPARA;

typedef struct{
    PIPARA Para;
    float I_out;
    float PI_out;
}PISTRU;

void PIControl(float error, PISTRU *p)
{
    p->I_out += p->Para.ki * p->Para.ts * error;
    if(p->I_out > p->Para.upLimit)
    {
        p->I_out = p->Para.upLimit;
    }
    else if(p->I_out < p->Para.dwLimit)
    {
        p->I_out = p->Para.dwLimit;
    }
    else
    {
        ;
    }

    p->PI_out = p->Para.kp * error + p->I_out;

    if(p->PI_out > p->Para.upLimit)
    {
        p->PI_out = p->Para.upLimit;
    }
    else if(p->PI_out < p->Para.dwLimit)
    {
        p->PI_out = p->Para.dwLimit;
    }
    else
    {
    }
}

