#include <SPI.h>
#include <RH_RF95.h>
#include "credentials.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0
#define LED 12

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
	Serial.begin(115200);
	
	// init led
	pinMode(LED, OUTPUT);

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
}

void loop()
{
	if (rf95.available())
	{
		uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
		uint8_t len = sizeof(buf);

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