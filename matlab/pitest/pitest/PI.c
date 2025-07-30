/*
 * PI.c
 *
 *  Created on: 2023Äê1ÔÂ8ÈÕ
 *      Author: Administrator
 */
#include <math.h>

typedef struct{
    double kp;
    double ki;
    double upLimit;
    double dwLimit;
    double ts;
}PIPARA;

typedef struct{
    PIPARA Para;
    double I_out;
    double PI_out;
}PISTRU;

void PIControl(double error, PISTRU *p)
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

