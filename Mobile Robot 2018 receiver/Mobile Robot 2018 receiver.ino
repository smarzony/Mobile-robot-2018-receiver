//Podsumowanie na dziœ: konwerter stanów logicznych jest wymagany do dzia³ania uk³adu radiowego


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ONE_MS 1

RF24 radio(26, 27);

const byte rxAddr[6] = {'1','N','o','d','e','0'};
byte incoming_message[10];

unsigned long long now, last_message_read;

void setup()
{
	Serial.begin(9600);

	radio.begin();
	radio.setChannel(110);
	radio.openReadingPipe(0, rxAddr);
	radio.startListening();
}

void loop()
{
	now = millis();
	//readRadio(ONE_MS);
	if (radio.available())
	{
		radio.read(&incoming_message, sizeof(incoming_message));
		printMessage();
	}
}

void readRadio(int period)
{
	if (now - last_message_read >= period)
	{
		if (radio.available())
		{
			radio.read(&incoming_message, sizeof(incoming_message));
			printMessage();
		}
	}
}

void printMessage()
{
	for (int i = 0; i < sizeof(incoming_message); i++)
	{
		Serial.print(incoming_message[i]);
		Serial.print(' ');
	}
	Serial.println();
}

