#ifndef IGVCMOTORH
#define IGVCMOTORH

class IGVCMotor {
    public:
        IGVCMotor(PinName enable, PinName pwm) {
            enablePin = new DigitalOut(enable);
            pwmPin = new AnalogOut(pwm);
        }
        
        void output(float speed) {
            if (speed < 0.05f) {
                setEnabled(0);
            } else {
                setEnabled(1);
            }
            if (speed > 1) {
                speed = 1;
            }
            pwmPin->write(speed);
        }
        
        void setEnabled(int enabled) {
            enablePin->write(enabled);
        }
        
        IGVCMotor& operator= (float v) {
            output(v);
            return *this;
        }
        
    private:
        DigitalOut* enablePin;
        AnalogOut* pwmPin;
        
};

#endif