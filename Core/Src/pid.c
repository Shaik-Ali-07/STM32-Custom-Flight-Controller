/*
 * pid.c
 *
 *  Created on: Nov 25, 2025
 *      Author: user
 */


#include "pid.h"

void PID_Init(PID *pid){

	//clear variables
	pid->differentiator = 0.0f;
	pid->integrator     = 0.0f;
	pid->prevError      = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out            = 0.0f;

}


float PID_Update(PID *pid, float setpoint, float measurement){

	//error signal
	float error = setpoint - measurement;

	//proportional
	float proportional = pid->Kp * error;


	//integral
	pid->integrator = pid->integrator + 0.5f * pid->T * pid->Ki * (error + pid->prevError);

	//anti windup
	float minlimInt, maxlimInt;

	//dynamic anti windup
	if(pid->maxlim > proportional){

	maxlimInt = pid->maxlim - proportional;

	}else{

		maxlimInt = 0.0f;

	}

	if(pid->minlim < proportional){

		minlimInt =  pid->minlim - proportional;

	}else{

		minlimInt = 0.0f;

	}

	//clamp integral
	if(pid->integrator > maxlimInt){

		pid->integrator = maxlimInt;

	}else if(pid->integrator < minlimInt){

		pid->integrator = minlimInt;

	}

	//derivative (band limit derivative)
	float denominator = (2.0f * pid->Tau + pid->T);
	if (denominator < 0.0001f) denominator = 0.0001f; // Safety check

	pid->differentiator = ((2.0f * pid->Kd * (measurement - pid->prevMeasurement))
	                    + (2.0f * pid->Tau - pid->T) * pid->differentiator)
	                    / denominator;

	//compute output
	pid->out = proportional + pid->integrator - pid->differentiator;

	if(pid->out > pid->maxlim){

		pid->out = pid->maxlim;

	}else if(pid->out < pid->minlim){

		pid->out = pid->minlim;

	}

	//update the previous
	pid->prevError       = error;
	pid->prevMeasurement = measurement;

	//return
	return pid->out;
}
