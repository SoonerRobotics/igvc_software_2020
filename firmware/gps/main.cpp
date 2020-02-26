#include "mbed.h"
#include "TinyGPSPlus.h"
#include "ArduinoJSON.h"

// Serial to ROS (9600)
Serial pc(USBTX,USBRX);            
// Serial from GPS (9600)
Serial gps_serial(PA_9, PA_10);     

TinyGPSPlus GPS;                   

float_t lat;
float_t lon;

char ret[256];

// Time spent sleeping every pass in millis
float_t waitTime = 20;

int main()
{
    gps_serial.baud(9600);
    
    StaticJsonDocument<256> doc;
    
    // Infinite loop that gets GPS data
    while(1) {
        
        if (GPS.encode(gps_serial.getc())) {
            
            // GPS has signal
            if (GPS.location.isValid()) {
                lat = GPS.location.lat();
                lon = GPS.location.lng();
                
                doc["latitude"] = lat;
                doc["longitude"] = lon;
                doc["hasSignal"] = true;
            }
            // GPS has no signal
            else {
                doc["latitude"] = 1000;
                doc["longitude"] = 1000;
                doc["hasSignal"] = false;
            }
            
            // Convert to JSON
            serializeJson(doc, ret);

            // Send via serial
            pc.printf("%s\r\n", ret);
            
            ThisThread::sleep_for(waitTime);
        }
    }
}