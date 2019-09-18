// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

#include "credentials.h"

/* for feather m0 RFM9x    */
    #define RFM95_CS 8
    #define RFM95_RST 4
    #define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// If the robot should be stopped, turn on this LED
#define LED 12


/**
 * Setup the device
 */
void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(115200);
    delay(100);

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    // If init fails then give up
    while (!rf95.init()){}

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // Set the board frequency
    while (!rf95.setFrequency(RF95_FREQ)){}

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
}


/**
 * Watchdog for the emergency stop and other messages
 */
void loop()
{
    // If there is any data available on the buffer
    if (rf95.available())
    {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        // Receive the data packet
        if (rf95.recv(buf, &len))
        {
            // Convert packet to character array to determine message contents
            char* msg = (char *)buf;

            // If the package is the password, emergency stop the robot
            if(strcmp(ROBOT_PASSWORD, msg) == 0)
            {
                digitalWrite(LED, HIGH);

                // Send acknowledgement
                uint8_t data[] = "ACK";
                rf95.send(data, sizeof(data));
                rf95.waitPacketSent();
            }
            // Otherwise, reject the message
            else
            {
                // Send a not-acknowledgement
                uint8_t data[] = "NACK";
                rf95.send(data, sizeof(data));
                rf95.waitPacketSent();
            }            
        }
        // Receive failure, notify user
        // TODO: put an LED on this
        else
        {
            Serial.println("Receive failed");
        }
    }
}