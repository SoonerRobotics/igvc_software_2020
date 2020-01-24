#include "mbed.h"
#include "TinyGPSPlus.h"
#include "ArduinoJSON.h"

Serial pc(USBTX,USBRX);             // To my computer
Serial gps_serial(PA_9, PA_10);     // To the GPS
TinyGPSPlus GPS;                    // Gps object

float_t lat;
float_t lon;

char ret[256];

int main()
{
    gps_serial.baud(9600);
    
    StaticJsonDocument<256> doc;
    
    while(1) {
        if (GPS.encode(gps_serial.getc())) {
            
            if (GPS.location.isValid()) {
                lat = GPS.location.lat();
                lon = GPS.location.lng();
                
                doc['latitude'] = lat;
                doc['longitude'] = lon;
                
                serializeJson(doc, ret);
                
                pc.printf("%s\r\n", ret);
            }
            
            ThisThread::sleep_for(500);
        }
    }
}