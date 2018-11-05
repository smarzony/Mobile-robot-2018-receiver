//Podsumowanie na dziœ: konwerter stanów logicznych jest wymagany do dzia³ania uk³adu radiowego


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(26,27);

const byte rxAddr[6] = "00001";

void setup()
{
	while (!Serial);
	Serial.begin(9600);

	radio.begin();
	radio.openReadingPipe(0, rxAddr);

	radio.startListening();
}

void loop()
{
	if (radio.available())
	{
		char text[32];
		radio.read(&text, sizeof(text));

		Serial.println(text);
	}
}