/*
 * pid.h
 *
 *  Created on: Nov 25, 2025
 *      Author: user
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct{

	float Kp;
	float Ki;
	float Kd;  //coefficients

	float Tau; //filter
	float T;   //sampling time

	//output limits
	float minlim;
	float maxlim;


	float integrator;
	float differentiator;
	float prevError;
	float prevMeasurement;

	float out;

}PID;

void PID_Init(PID *pid);
float PID_Update(PID *pid, float setpoint, float measurement);

#endif /* INC_PID_H_ */
