void serialPrint(int period)
{
	if (now - last_serial_print > period)
	{
		bool any_print = false;
		last_serial_print = now;
		/*Serial.print("SV: ");
		Serial.print(shunt_voltage_plus_ADC - shunt_voltage_minus_ADC);
		Serial.print("\tCurrent: ");
		Serial.print(current_shunt);*/

		if (average_current > 900)
		{
			any_print = true;
			Serial.print("I: ");
			Serial.print(average_current);
			Serial.print(" mA");
		}

	/*	Serial.print("\tT : ");
		Serial.print(temperature);
		Serial.print(" *C");
	*/
		if (messages_lost > 1)
		{
			any_print = true;
			Serial.print("\tData loss: ");
			Serial.print(messages_lost);
		}

		if (any_print)
			Serial.println();
		if (radio_not_availalble)
		{
			Serial.print(radio_not_availalble_counter);
			Serial.println(". Radio not available");
		}
	}
}
