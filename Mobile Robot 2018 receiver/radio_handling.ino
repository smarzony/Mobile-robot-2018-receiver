void radioConfig()
{
	/*
	digitalWrite(CE, 0);
	digitalWrite(CSN, 0);
	delay(50);
	*/
	radio.begin();
	radio.setDataRate(RF24_1MBPS);
	radio.setChannel(0);//100	
	// ------------   JAK SIÊ ROZJEBIE TO ZMIEN KANA£ -----
	radio.openReadingPipe(0, rxAddr);
	radio.openWritingPipe(txAddr);
	radio.startListening();
}

void sendRadio()
{		
	radio.stopListening();
	radio.write(&message_transmit, sizeof(message_transmit));
	radio.startListening();

	message_transmit.message_no++;
}

void readRadio(int period, bool printing)
{

	if ((now - last_message_read >= period))
	{		
		if (radio.available())
		{			
			last_message_no = current_message_no;
			
			radio.read(&message_receive, sizeof(message_receive));
			current_message_no = message_receive.message_no;
			messages_lost = current_message_no - last_message_no - 1;
			if (printing)
				printMessage(RADIO_PRINT_INCOMING_MESSAGE);
			radio_not_availalble = 0;
			radio_not_availalble_counter = 0;

			side_switch = bitRead(message_receive.bit_array, 0);
			analog_left_switch = bitRead(message_receive.bit_array, 1);
			analog_right_switch = bitRead(message_receive.bit_array, 2);
			rotory_encoder_switch = bitRead(message_receive.bit_array, 3);
		}
		else
		{
			radio_not_availalble = 1;
			radio_not_availalble_counter++;
		}

		if (message_receive.analog_left_Y == 0 && message_receive.analog_left_X &&
			message_receive.analog_right_X == 0 & message_receive.analog_right_Y == 0)
			empty_receive_data = 1;
		else
			empty_receive_data = 0;

	}
}

void printMessage(bool print_message)
{
	if (print_message)
	{/*
		for (int i = 0; i < sizeof(incoming_message); i++)
		{
			Serial.print(incoming_message[i]);
			Serial.print(' ');
		}*/

		Serial.print(message_receive.analog_left_X);
		Serial.print(' ');
		Serial.print(message_receive.analog_left_Y);
		Serial.print(' ');
		Serial.print(message_receive.analog_right_X);
		Serial.print(' ');
		Serial.print(message_receive.analog_right_Y);
		Serial.print(' ');
		Serial.print(bitRead(message_receive.bit_array, 0));
		Serial.print(bitRead(message_receive.bit_array, 1));
		Serial.print(bitRead(message_receive.bit_array, 2));
		Serial.print(bitRead(message_receive.bit_array, 3));
		Serial.print(' ');
		Serial.print(message_receive.rotory_encoder);
		Serial.print(' ');
		Serial.print(message_receive.message_no);

		Serial.println();

	}

}