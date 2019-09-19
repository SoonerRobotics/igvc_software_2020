#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "credentials.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 9
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 6, 10, 11, 12);

volatile bool sendStop = false;

void ISR()
{
	sendStop = true;
}

void setup()
{
	Serial.begin(115200);

	// display init to blank screen
	display.begin();
	display.setContrast(50);
	display.setReinitInterval(10);
	display.clearDisplay();
	display.display();

	// reset rfm95
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	// init rfm95
	rf95.init();
	rf95.setFrequency(RF95_FREQ);
	rf95.setTxPower(23, false);

	// button interrupt setup
	pinMode(STOP_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(STOP_PIN), ISR, FALLING);
}

void loop()
{
	while(sendStop)
	{
		// display text press
		display.clearDisplay();
		display.setTextSize(1);
		display.setTextColor(BLACK);
		display.setCursor(0,0);
		display.print("Sent kill!");
		display.display();
		
		// send packet
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
