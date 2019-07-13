#include "igvc_nav/robotlib/PIDController.h"

/*
    
*/
PIDController::PIDController()
{
    this->last_process_var = 0;
    this->err = 0;
    this->integrator = 0;
    this->prevErr = 0;

    this->kP = 0;
    this->kI = 0;
    this->kD = 0;

    this->low = -1;
    this->high = 1;	
}

/*
    
*/
PIDController::PIDController(float process_init, float kp, float ki, float kd)
{
    this->last_process_var = process_init;
    this->err = 0;
    this->prevErr = 0;
    this->integrator = 0;

    this->kP = kp;
    this->kI = ki;
    this->kD = kd;

    this->low = -1;
    this->high = 1;
}

/*
    Copies over another PIDController

    @param pidController - PIDController object that gets copied
*/
void PIDController::operator=(const PIDController& pidController)
{
    this->last_process_var = pidController.last_process_var;
    this->kP = pidController.kP;
    this->kI = pidController.kI;
    this->kD = pidController.kD;
    this->err = pidController.err;
    this->prevErr = pidController.prevErr;
    this->integrator = pidController.integrator;
    this->low = pidController.low;
    this->high = pidController.high;
}

void PIDController::initialize(float process_init, float kp, float ki, float kd)
{
    // the initial process variable data should be saved for the first run
    this->last_process_var = process_init;
	
    // set the PID constants
    this->kP = kp;
    this->kI = ki;
    this->kD = kd;

    // initial error and cumulative error can start at 0
    this->err = 0;
    this->integrator = 0;
	
    // initialize the start time in seconds
    lastTime = ros::Time::now().toNSec() / 1000.0 / 1000.0 / 1000.0;
}


/**
 * @brief Gets the output from the PID
 * 
 * @param setpoint - The target value
 * @param cur_var - The current value
 * @return double - The output (p + i + d)
 */
float PIDController::getOutput(float setpoint, float process)
{
    // Find the error
    this->err = setpoint - process;

    // Apply Proportional control
    float P = kP * err;

    // Get the current time and find the timestep between loops
    double cur_time = ros::Time::now().toNSec() / 1000.0 / 1000.0 / 1000.0;
    float dT = lastTime - cur_time;

    // Find cumulative error through trapezoidal Riemann sum approximation
    this->prevErr = setpoint - last_process_var;
    this->integrator += 0.5 * (err + prevErr) * dT;

    // Apply Integral control
    float I = this->kI * this->integrator;

    // Find the slope of the error curve
    float slope = (process - this->last_process_var)/dT;

    // Apply Derivative control
    float D = this->kD * slope;

    // Sum Proportional, Integral, and Derivative control. Clamp the output to maxes and mins
    float output = RLUtil::clamp(P + I + D, this->high, this->low);

    // Save the end time of this loop and this process data point
    this->last_process_var = process;
    this->lastTime = cur_time;

    // Return PID output
    return output;
}

void PIDController::setOutputRange(float upper, float lower)
{
    this->high = upper;
    this->low = lower;
}
