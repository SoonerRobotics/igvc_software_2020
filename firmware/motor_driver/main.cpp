#include "mbed.h"
#include "ArduinoJson.h"
#include "IGVCMotor.h"
#include <string> 

IGVCMotor motorLeft(PB_7, PA_4);
IGVCMotor motorRight(PB_6, PA_5);

Serial pc(SERIAL_TX, SERIAL_RX);

int main()
{
    pc.baud(9600);
    wait(0.01);
        
    StaticJsonDocument<256> doc;
    
    while (1) {
        char rawJson[256];
        if (pc.readable()) {
            pc.scanf("%s", rawJson);
            DeserializationError err = deserializeJson(doc, rawJson);

            if (!err) {
                
                const char* message = doc["igvc"];
                
                if (strcmp(message, "yo") == 0) {
                    pc.printf("{\"id\":\"motors\"}\n");
                }
                
                float motorLeftInstruction = doc["motorLeft"];
                float motorRightInstruction = doc["motorRight"];
                
                motorLeft = motorLeftInstruction;
                motorRight = motorRightInstruction;
            }
        }
    }
}
