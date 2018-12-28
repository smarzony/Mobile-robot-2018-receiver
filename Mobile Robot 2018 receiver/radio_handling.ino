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

void readRadio(int period)
{

	if (now - last_message_read >= period)
	{
		///Serial.println("In function");
		if (radio.available())
		{
			//Serial.println("Radio available");
			last_message_no = current_message_no;
			radio.read(&incoming_message, sizeof(incoming_message));
			current_message_no = incoming_message[MESSAGE_NO];
			messages_lost = current_message_no - last_message_no - 1;
			printMessage(RADIO_PRINT_INCOMING_MESSAGE);
			radio_not_availalble = 0;
		}
		else
		{
			radio_not_availalble = 1;
			
		}

	}
}

void printMessage(bool print_message)
{
	if (print_message)
	{
		for (int i = 0; i < sizeof(incoming_message); i++)
		{
			Serial.print(incoming_message[i]);
			Serial.print(' ');
		}
		Serial.println();
	}

}