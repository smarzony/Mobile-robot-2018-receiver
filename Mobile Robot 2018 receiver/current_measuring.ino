void CircBuffer(short cBuffer[], short push_input, short buf_length)
{
	for (short i = buf_length - 1; i >= 0; i--)
	{
		cBuffer[i] = cBuffer[i - 1];
	}
	cBuffer[0] = push_input;
}

void floatCircBuffer(float cBuffer[], float push_input, short buf_length)
{
	for (short i = buf_length - 1; i >= 0; i--)
	{
		cBuffer[i] = cBuffer[i - 1];
	}
	cBuffer[0] = push_input;
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void shunt_measure(int period)
{

	if (now - last_shunt_measure > period)
	{
		last_shunt_measure = now;
		int shunt_voltage_plus_ADC = analogRead(SHUNT_PIN_PLUS);
		int shunt_voltage_minus_ADC = analogRead(SHUNT_PIN_MINUS);
		current_shunt = map(shunt_voltage_plus_ADC - shunt_voltage_minus_ADC, 0, 1023, 0, 50000);

		float sum = 0.0;
		floatCircBuffer(current_array, current_shunt, CURRENT_BUFFER_SIZE);

		if (now >= period * CURRENT_BUFFER_SIZE)
		{
			for (int i = 0; i < CURRENT_BUFFER_SIZE - 1; i++)
			{
				sum = sum + current_array[i];
			}
			average_current = sum / CURRENT_BUFFER_SIZE;
		}

	}
}
