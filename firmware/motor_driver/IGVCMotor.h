#ifndef IGVCMOTORH
#define IGVCMOTORH

class IGVCMotor {
    public:
        IGVCMotor(PinName a1, PinName a2, PinName pwm) {
            this -> a1 = new DigitalOut(a1);
            this -> a2 = new DigitalOut(a2);
            this -> pwmPin = new PwmOut(pwm);
            
            this -> pwmPin -> period(1.0/15000.0); // 15 kHz frequency
        }
        
        void output(float speed) {
            if (speed < 0.05f && speed > -0.05f) { // break
                a1 -> write(0);
                a2 -> write(0);
                pwmPin->write(0);
            } else if (speed > 0.0f) { // forward
                a1 -> write(0);
                a2 -> write(1);
                pwmPin->write(speed);
            } else { // reverse
                a1 -> write(1);
                a2 -> write(0);
                pwmPin->write(-speed); // mbed only accepts positive PWM duty cycles
            }
        }
        
        IGVCMotor& operator= (float v) {
            output(v);
            return *this;
        }
        
    private:
        DigitalOut* a1;
        DigitalOut* a2;
        PwmOut* pwmPin;
        
};

#endif