#include "mbed.h"
#include "ArduinoJson.h"
#include "IGVCMotor.h"

IGVCMotor motorLeft(PB_6, PA_4);
IGVCMotor motorRight(PB_7, PA_5);

Serial pc(SERIAL_TX, SERIAL_RX);

int main()
{
    pc.baud(115200);
    wait(0.01);
        
    StaticJsonDocument<256> doc;
    
    while (1) {
        char rawJson[256];
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
