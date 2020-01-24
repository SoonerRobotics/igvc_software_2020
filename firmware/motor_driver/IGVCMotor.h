#ifndef IGVCMOTORH
#define IGVCMOTORH

#define MOTOR_UPDATE_RATE 40 // Frequency that motor PID is updated (Hz)
#define MAX_SPEED 2.2f // (m/s)
#define PULSES_PER_REV 2400 // (revs)
#define LINEAR_PER_REV 0.254f // Wheel radius (m)
#define PI 3.14159265f

#define PREV_MASK 0x1 // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 // Mask for the current state in determining direction of rotation.
#define INVALID   0x3 // XORing two states where both bits have changed.

class IGVCMotor {
    public:
        IGVCMotor(PinName motorA, PinName motorB, PinName pwm){
            this->motorA = new DigitalOut(motorA);
            this->motorB = new DigitalOut(motorB);
            this->pwmPin = new PwmOut(pwm);
            
            this->pwmPin->period(1.0/15000.0); // 15 kHz frequency
            
            this->pulses = 0;
            this->targetSpeed = 0.0f;
            this->curOutput = 0.0f;
            
            // PID values
            this->integrator = 0.0f;
            this->error = 0.0f;
            
            // States
            this->prev_error = 0.0f;
            this->last_state = 0.0f;
            
            // Equation constants
            this->kP = 0.005f;
            this->kI = 0.0f;
            this->kD = 0.00005f;
        }
        
        void output(float speed) {
            this->targetSpeed = speed;
        }
        
        // Stolen from QEI library
        void pulse(int left, int right) {
            currState_ = (left << 1) | (right);
            
            //Entered a new valid state.
            if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_)) {
                //2 bit state. Right hand bit of prev XOR left hand bit of current
                //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
                int change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);
    
                if (change == 0) {
                    change = -1;
                }
    
                pulses -= change;
            }
            
            prevState_ = currState_;
        }
        
        // Poll to update PID
        void update() {
            float output = this->updatePID(this->targetSpeed, this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE);
            curOutput += output;
            curOutput = clamp(curOutput, -1.0, 1.0);

            //printf("%f,%f,%f\n\r", this->targetSpeed, this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE, curOutput);
            
            this->pulses = 0;
            
            if (curOutput < 0.05f && curOutput > -0.05f) { // break
                motorA->write(0);
                motorB->write(0);
                pwmPin->write(0.0f);
            } else if (curOutput > 0.0f) { // forward
                motorA->write(0);
                motorB->write(1);
                pwmPin->write(curOutput);
            } else { // reverse
                motorA->write(1);
                motorB->write(0);
                pwmPin->write(-curOutput);
            }  
        }
        
        void tuneP(float p) {
            this->kP = p;
        }
        
        void tuneI(float i) {
            this->kI = i;
        }
        
        void tuneD(float d) {
            this->kD = d;
        }
        
        IGVCMotor& operator= (float v) {
            output(v);
            return *this;
        }
        
    private:
        DigitalOut* motorA;
        DigitalOut* motorB;
        PwmOut* pwmPin;
        
        int prevState_;
        int currState_;
        volatile int pulses;
        
        float curOutput;
        float targetSpeed;
        
        // PID Values
        float integrator;
        float error;

        // States
        float prev_error;
        float last_state;

        // Equation coefficients
        float kP;
        float kI;
        float kD;
        
        // Adapted (Stolen) from RobotLib :)
        float updatePID(float target_state, float cur_state) {            
            // Declare local variables
            float P, I, D;
            float result;
            float slope;
            float dt;
            
            // Get the time step
            dt = 1.0f/MOTOR_UPDATE_RATE;
            
            // Calculate error
            this->error = target_state - cur_state;
            
            // Integrate error using trapezoidal Riemann sums
            this->prev_error = target_state - this->last_state;
            this->integrator += 0.5f * (this->error + this->prev_error) * dt;
            
            // Find the slope of the error curve using secant approximation
            slope = (cur_state - this->last_state) / dt;
            
            // Apply PID gains
            P = this->kP * this->error;
            I = this->kI * this->integrator;
            D = this->kD * slope;
            
            // Sum P, I, D to get the result of the equation
            // Bind the output if needed
            result = clamp(P + I + D, -.6f, .6f);
            
            // Update timing and increment to the next state
            this->last_state = cur_state;
            
            // Return the PID result
            return result;
        }
        
        static float clamp(float val, float min, float max)
        {
            if(val > max)
            {
                return max;
            }
            else if(val < min)
            {
                return min;
            }
            return val;
        }
};

#endif