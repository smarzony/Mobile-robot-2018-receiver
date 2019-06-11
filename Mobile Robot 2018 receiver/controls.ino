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
				LeftMotor.backward(velocityLimit);

				break;
			case 91 ... 180:
				LeftMotor.stop();
				break;
			case 181 ... 255:
				LeftMotor.forward(velocityLimit);
				break;
			}

			switch (message.analog_right_Y)
			{
			case 0 ... 90:
				RightMotor.backward(velocityLimit);
				break;
			case 91 ... 180:
				RightMotor.stop();
				break;
			case 181 ... 255:
				RightMotor.forward(velocityLimit);
				break;
			}
			break;

		case CONTROLS_ENCHANCED:
			
			if (message.analog_left_Y >= 0 && message.analog_left_Y < (128 - dead_zone))
			{			
				speed_general = map(message.analog_left_Y, 1, 85, velocityLimit, 1);
				direction = BWD;			
			}
			if (message.analog_left_Y >= (128 - dead_zone) && message.analog_left_Y < (128 + dead_zone))
			{		
				speed_general = 0.0;				
			}
			if (message.analog_left_Y >= (128 + dead_zone) && message.analog_left_Y <= 255)
			{			
				speed_general = map(message.analog_left_Y - 171, 1, 85, 1, velocityLimit);
				direction = FWD;				
			}
			

			
			if (message.steering_wheel >= 0 && message.steering_wheel < (128 - dead_zone))
			{			
				steer_left = map(message.steering_wheel, 0, (128 - dead_zone), 0, 100);
			}
			if (message.steering_wheel >= (128 - dead_zone) && message.steering_wheel < (128 + dead_zone))
			{			
				steer_left = 100.0;
				steer_right = 100.0;				
			}
			if (message.steering_wheel >= (128 + dead_zone) && message.steering_wheel <= 255)
			{
				steer_right = map(message.steering_wheel, (128 + dead_zone), 255, 100, 0);
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
				LeftMotor.backward((short)speed_left);
				RightMotor.backward((short)speed_right);
			}
			break;

		case CONTROLS_MEASURED:
			// SPEED
			if (message.analog_left_Y >= 0 && message.analog_left_Y < (128 - dead_zone))
			{
				speed_general = map(message.analog_left_Y, 1, (128 - dead_zone), velocityLimit, 1);
				direction = BWD;			
			}
			if (message.analog_left_Y >= (128 - dead_zone) && message.analog_left_Y < (128 + dead_zone))
			{				
				speed_general = 0.0;					
			}
			if (message.analog_left_Y >= (128 + dead_zone) && message.analog_left_Y <= 255)
			{		
				speed_general = map(message.analog_left_Y - (128 + dead_zone), 1, (128 - dead_zone), 1, velocityLimit);
				direction = FWD;			
			}

			// STEERING
			
			if (message.analog_right_X >= 0 && message.analog_right_X < (128 - dead_zone))
			{
				steer_left = map(message.analog_right_X, 0, 128 - dead_zone, 0, 100);
				steer_right = 100.0;
			}

			if (message.analog_right_X >= (128 - dead_zone) && message.analog_right_X < (128 + dead_zone))
			{
				steer_left = 100.0;
				steer_right = 100.0;
			}

			if (message.analog_right_X >= (128 + dead_zone) && message.analog_right_X <= 255)
			{
				//steer_right = map(message.analog_right_X, (128 + dead_zone), 255, 100, 0);
				steer_right = float((message.analog_right_X - 145) * (-10) + 100);
				steer_left = 100.0;
			}
			


			speed_left = speed_general *steer_left / 100;
			speed_right = speed_general *steer_right / 100;

			if (direction == FWD)
			{
				LeftMotor.forward((short)PWM_left_motor);
				RightMotor.forward((short)PWM_right_motor);
			}
			else if (direction == BWD)
			{
				LeftMotor.backward((short)PWM_left_motor);
				RightMotor.backward((short)PWM_right_motor);
			}
			break;

		default:
			LeftMotor.stop();
			RightMotor.stop();
		}

	}
}

void computePWM()
{
	errorLeft = (int)measured_speed_left - (int)speed_left;
	errorRight = (int)measured_speed_right - (int)speed_right;
	switch (errorLeft)
	{
		case (-999)...(-100) :
			PWM_left_motor += 20;
			break;
		case (-99)...(-50) :
			PWM_left_motor += 15;
			break;
		case (-49)... (-30) :
			PWM_left_motor += 12;
			break;
		case (-29)... (-15) :
			PWM_left_motor += 8;
			break;
		case (-14)... (-6) :
			PWM_left_motor += 5;
			break;
		case (-5)... (-2) :
			PWM_left_motor += 1;
			break;
		case (-1)... (1) :
			//PWM_left_motor += 0;
			break;
		case (2)... (5) :
			PWM_left_motor += -1;
			break;
		case (6)... (14) :
			PWM_left_motor += -5;
			break;
		case (15)... (29) :
			PWM_left_motor += -8;
			break;
		case (30)... (49) :
			PWM_left_motor += -12;
			break;
		case (50)... (999) :
			PWM_left_motor += -15;
			break;
	}

	if (PWM_left_motor > 255)
		PWM_left_motor = 255;
	if (PWM_left_motor < 0)
		PWM_left_motor = 0;

	switch (errorRight)
	{
		case (-999)...(-100) :
			PWM_right_motor += 20;
			break;
		case (-99)...(-50) :
			PWM_right_motor += 15;
			break;
		case (-49)... (-30) :
			PWM_right_motor += 12;
			break;
		case (-29)... (-15) :
			PWM_right_motor += 8;
			break;
		case (-14)... (-6) :
			PWM_right_motor += 5;
			break;
		case (-5)... (-2) :
			PWM_right_motor += 1;
			break;
		case (-1)... (1) :
			//PWM_left_motor += 0;
			break;
		case (2)... (5) :
			PWM_right_motor += -1;
			break;
		case (6)... (14) :
			PWM_right_motor += -5;
			break;
		case (15)... (29) :
			PWM_right_motor += -8;
			break;
		case (30)... (49) :
			PWM_right_motor += -12;
			break;
		case (50)... (999) :
			PWM_right_motor += -25;
			break;
	}

	if (PWM_right_motor > 255)
		PWM_right_motor = 255;
	if (PWM_right_motor < 0)
		PWM_right_motor = 0;
}

void computePID()
{
	errorLeft = ((int)speed_left - (int)measured_speed_left);
	errorRight = ((int)speed_right - (int)measured_speed_right);

	integralLeft += (((float)errorLeft + (float)errorLeft_last) / 2)*PID_DT/1000;
	integralRight += (((float)errorRight + (float)errorRight_last) / 2)*PID_DT/1000;
	if (abs(speed_general) < 1.0)
	{
		integralLeft = 0.0;
		integralRight = 0.0;
	}
	
	//integralLeft = 0.0;
	//integralRight = 0.0;


	if (integralLeft > integralLimit)
		integralLeft = integralLimit;
	if (integralLeft < -integralLimit)
		integralLeft = -integralLimit;

	if (integralRight > integralLimit)
		integralRight = integralLimit;
	if (integralRight < -integralLimit)
		integralRight = -integralLimit;

	differentialLeft = errorLeft - errorLeft_last;
	differentialRight = errorRight - errorRight_last;

	PWM_left_motor = Kp * (errorLeft + ((1 / Ki)*integralLeft ) + (Kd * differentialLeft / PID_DT));
	PWM_right_motor = Kp * (errorRight + ((1 / Ki)*integralRight ) + (Kd *differentialRight / PID_DT));

	if (PWM_left_motor > 255.0)
		PWM_left_motor = 255.0;
	if (PWM_left_motor < 0.0)
		PWM_left_motor = 0.0;

	if (PWM_right_motor > 255.0)
		PWM_right_motor = 255.0;
	if (PWM_right_motor < 0.0)
		PWM_right_motor = 0.0;

	errorLeft_last = errorLeft;
	errorRight_last = errorRight;
}

