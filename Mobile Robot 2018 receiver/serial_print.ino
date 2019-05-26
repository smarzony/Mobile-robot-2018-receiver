
void serialPrintStandard()
{

	Serial.println("STANDARD");
	bool any_print = false;
	last_serial_print = now;
	Serial.print("SV: ");
	//Serial.print(shunt_voltage_plus_ADC - shunt_voltage_minus_ADC);
	Serial.print("\tCurrent: ");
	Serial.print(current_shunt);

	if (average_current > 900)
	{
		any_print = true;
		Serial.print("I: ");
		Serial.print(average_current);
		Serial.print(" mA");
	}

	//Serial.print("\tT : ");
	//Serial.print(temperature);
	//Serial.print(" *C");

	if (messages_lost > 1)
	{
		any_print = true;
		Serial.print("\tData loss: ");
		Serial.print(messages_lost);
	}

	if (any_print)
		Serial.println();
	/*if (!radio.available())
	{
		Serial.print(radio_not_availalble_counter);
		Serial.println(". Radio not available");
	}*/

	if (!radio.isChipConnected())
		Serial.println("\nRadio disconnected");

	if (!radio.isValid())
		Serial.println("\nRadio not valid");

}
void serialPrintSpeedMonitor()
{
	String output;
	output = "L_Y:";
	output += message.analog_left_Y;
	output += " R_X:";
	output += message.analog_right_X;

	if (direction == FWD)
		output += " FWD";
	else if (direction == BWD)
		output += " BWD";

	output += " S_G:";
	output += speed_general;
	output += " S_L:";
	output += speed_left;
	output += " S_R:";
	output += speed_right;
	output += " St_L:";
	output += steer_left;
	output += " St_R:";
	output += steer_right;

	Serial.println(output);

}
