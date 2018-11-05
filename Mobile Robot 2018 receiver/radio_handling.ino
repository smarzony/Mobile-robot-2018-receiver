//
//
//void sendData(int period)
//{
//	if (now - last_radio_transmit >= period)
//	{
//		Serial.println("send radio");
//		last_radio_transmit = now;
//		dataTransmitted[0] = analogRead(SHUNT_PIN_PLUS) - analogRead(SHUNT_PIN_MINUS);
//		dataTransmitted[1] = analogRead(SHUNT_PIN_PLUS) / 4;
//		Serial.println("stop listening");
//		myRadio.stopListening();
//		Serial.println("radio write");
//		myRadio.write(&dataTransmitted, sizeof(dataTransmitted));
//		Serial.println("start listening");
//		myRadio.startListening();
//	}
//}
//
//void displayData()
//{
//	Serial.print("Data Received = ");
//	for (int i = 0; i < sizeof(dataReceived); i++)
//	{
//		Serial.print(dataReceived[i]);
//		Serial.print(' ');
//	}
//	Serial.println();
//}
