// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "credentials.h"

/* for feather m0 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 9

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 10, 11, 12);

volatile int16_t packetnum = 0; // packet counter, we increment per xmission
volatile bool sendStop = false;

void ISR()
{
  sendStop = true;
}

void setup()
{
      display.begin();
    // init done
  
    // you can change the contrast around to adapt the display
    // for the best viewing!
    display.setContrast(50);
    display.setReinitInterval(10);
  
    display.clearDisplay();   // clears the screen and buffer
    display.display(); // show splashscreen
    
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    // ISR on pin 12
    pinMode(STOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), ISR, FALLING);

    Serial.begin(115200);

    delay(100);


    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!rf95.init())
    {
      delay(10);
    }

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ))
    {
        delay(10);
    }
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);

}

void loop()
{
  while(sendStop)
  {
    // text display tests
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.print("Sent kill!");
    display.display();
    
    Serial.println("Press!");
    //delay(1000);
    rf95.send((uint8_t *)ROBOT_PASSWORD, sizeof(ROBOT_PASSWORD));

    delay(10);
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(1000))
      if(rf95.recv(buf, &len))
        sendStop = false;
  }
}
