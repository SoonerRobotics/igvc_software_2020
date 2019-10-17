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
                pwmPin->write(0);
            if (speed <= 1.0f) {
                setEnabled(1);
                pwmPin->write(speed);
            } else {
                setEnabled(1);
                pwmPin->write(1);
            }
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