#include "mbed.h"
#include "ArduinoJson.h"
#include "IGVCMotor.h"

InterruptIn encoderLeftA(PB_5);
InterruptIn encoderLeftB(PB_4);
IGVCMotor motorLeft(PB_0, PB_7, PA_3);

InterruptIn encoderRightA(PA_8);
InterruptIn encoderRightB(PA_11);
IGVCMotor motorRight(PB_6, PB_1, PA_1);

Serial pc(SERIAL_TX, SERIAL_RX);
Ticker ticker;

void updateLeft() {
    motorLeft.pulse(encoderLeftA.read(), encoderLeftB.read());
}

void updateRight() {
    motorRight.pulse(encoderRightA.read(), encoderRightB.read());
}

void tick() {
    motorLeft.update();
    motorRight.update();
}

int main()
{
    // create poll
    ticker.attach(&tick, 1.0/MOTOR_UPDATE_RATE);
    
    // attach all interrupts
    encoderLeftA.rise(&updateLeft);
    encoderLeftA.fall(&updateLeft);
    encoderLeftB.rise(&updateLeft);
    encoderLeftB.fall(&updateLeft);
    encoderRightA.rise(&updateRight);
    encoderRightA.fall(&updateRight);
    encoderRightB.rise(&updateRight);
    encoderRightB.fall(&updateRight);
    
    pc.baud(115200);
    wait(0.01);
        
    StaticJsonDocument<256> doc;
    char rawJson[256];
    
    while (1) {
        if (pc.readable()) {
            pc.scanf("%s", rawJson);
            DeserializationError err = deserializeJson(doc, rawJson);

            if (!err) { 
                float motorLeftInstruction = doc["motorLeft"];
                float motorRightInstruction = doc["motorRight"];
                
                motorLeft = motorLeftInstruction;
                motorRight = motorRightInstruction;             
            }
        }
    }
}