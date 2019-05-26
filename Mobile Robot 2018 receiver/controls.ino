void controls(byte control_type)
{
	if (message.message_no != 0 && !empty_receive_data)
	{
		
		switch (control_type)
		{
		case CONTROLS_STANDARD:


			switch (message.analog_left_Y)
			{
			case 0 ... 90:
				LeftMotor.backward(80);

				break;
			case 91 ... 180:
				LeftMotor.stop();
				break;
			case 181 ... 255:
				LeftMotor.forward(80);
				break;
			}

			switch (message.analog_right_Y)
			{
			case 0 ... 90:
				RightMotor.backward(80);
				break;
			case 91 ... 180:
				RightMotor.stop();
				break;
			case 181 ... 255:
				RightMotor.forward(80);
				break;
			}
			break;

		case CONTROLS_ENCHANCED:
			switch (message.analog_left_Y)
			{
			case 0 ... 85:
				speed_general = map(message.analog_left_Y, 1, 85, 120, 1);				
				direction = BWD;
				break;

			case 86 ... 170:
				speed_general = 0.0;
				break;

			case 171 ... 255:
				speed_general = map(message.analog_left_Y - 171, 1, 85, 1, 120);				
				direction = FWD;
				break;
			}

			switch (message.analog_right_X)
			{
			case 0 ... 85:
				steer_left = map(message.analog_right_X, 0, 85, 0, 100);
				
				break;

			case 86 ... 170:
				steer_left = 100.0;
				steer_right = 100.0;
				break;

			case 171 ... 255:

				steer_right = map(message.analog_right_X, 171, 255, 100, 0);
				break;
			}

			speed_left = speed_general * steer_left / 100;
			speed_right = speed_general * steer_right / 100;

			if (direction == FWD)
			{
				LeftMotor.forward((short)speed_left);
				RightMotor.forward((short)speed_right);
			}
			else if (direction == BWD)
			{
				LeftMotor.backward(speed_left);
				RightMotor.backward(speed_right);
			}
			break;
		}

	}
}