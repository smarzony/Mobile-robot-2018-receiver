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
	// ------------   JAK SI� ROZJEBIE TO ZMIEN KANA� -----
	radio.openReadingPipe(0, rxAddr);
	radio.openWritingPipe(txAddr);
	radio.startListening();
}

void sendRadio()
{		
	message_transmit.velocity_measured_left = (uint8_t)measured_speed_left;
	message_transmit.velocity_measured_right = (uint8_t)measured_speed_right;
	message_transmit.distance = distance_measured;
	message_transmit.control_mode = control_mode;
	message_transmit.azimuth1 = machine_state.azimuth & 0xFF;
	message_transmit.azimuth2 = (machine_state.azimuth >> 8) & 0xFF;

	message_transmit.pos_lat[0] = position_latitude.data[0];
	message_transmit.pos_lat[1] = position_latitude.data[1];
	message_transmit.pos_lat[2] = position_latitude.data[2];
	message_transmit.pos_lat[3] = position_latitude.data[3];

	message_transmit.pos_long[0] = position_longtitude.data[0];
	message_transmit.pos_long[1] = position_longtitude.data[1];
	message_transmit.pos_long[2] = position_longtitude.data[2];
	message_transmit.pos_long[3] = position_longtitude.data[3];



	// message_transmit.longtitude = machine_state.longtitude;



	if ((now - RadioTimeoutTimer) < 255)
	{
		message_transmit.time_delay = uint8_t((now - RadioTimeoutTimer));
	}
	else
		message_transmit.time_delay = 255;

	radio.stopListening();
	radio.write(&message_transmit, sizeof(message_transmit));
	radio.startListening();

	message_transmit.message_no++;
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
		Serial.print(message_receive.message_no);

		Serial.println();

	}

}