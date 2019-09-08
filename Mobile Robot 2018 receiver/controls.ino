void controls(byte control_type)
{
	if (message_receive.message_no != 0 && !empty_receive_data)
	{
		
		switch (control_type)
		{
		case CONTROLS_STANDARD:
			control_mode = CONTROLS_STANDARD;

			switch (message_receive.analog_left_Y)
			{
			case 0 ... 90:
				LeftMotor.backward(VELOCITY_LIMIT);

				break;
			case 91 ... 180:
				LeftMotor.stop();
				break;
			case 181 ... 255:
				LeftMotor.forward(VELOCITY_LIMIT);
				break;
			}

			switch (message_receive.analog_right_Y)
			{
			case 0 ... 90:
				RightMotor.backward(VELOCITY_LIMIT);
				break;
			case 91 ... 180:
				RightMotor.stop();
				break;
			case 181 ... 255:
				RightMotor.forward(VELOCITY_LIMIT);
				break;
			}
			break;

		case CONTROLS_ENCHANCED:
			control_mode = CONTROLS_ENCHANCED;
			
			if (message_receive.analog_left_Y >= 0 && message_receive.analog_left_Y < (128 - DEAD_ZONE))
			{			
				speed_general = map(message_receive.analog_left_Y, 1, 85, VELOCITY_LIMIT, 1);
				direction = BWD;			
			}
			if (message_receive.analog_left_Y >= (128 - DEAD_ZONE) && message_receive.analog_left_Y < (128 + DEAD_ZONE))
			{		
				speed_general = 0.0;				
			}
			if (message_receive.analog_left_Y >= (128 + DEAD_ZONE) && message_receive.analog_left_Y <= 255)
			{			
				speed_general = map(message_receive.analog_left_Y - 171, 1, 85, 1, VELOCITY_LIMIT);
				direction = FWD;				
			}
			

			
			if (message_receive.steering_wheel >= 0 && message_receive.steering_wheel < (128 - DEAD_ZONE))
			{			
				steer_left = map(message_receive.steering_wheel, 0, (128 - DEAD_ZONE), 0, 100);
			}
			if (message_receive.steering_wheel >= (128 - DEAD_ZONE) && message_receive.steering_wheel < (128 + DEAD_ZONE))
			{			
				steer_left = 100.0;
				steer_right = 100.0;				
			}
			if (message_receive.steering_wheel >= (128 + DEAD_ZONE) && message_receive.steering_wheel <= 255)
			{
				steer_right = map(message_receive.steering_wheel, (128 + DEAD_ZONE), 255, 100, 0);
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
			control_mode = CONTROLS_MEASURED;
			// SPEED
			if (message_receive.analog_left_Y >= 0 && message_receive.analog_left_Y < (128 - DEAD_ZONE))
			{
				speed_general = map(message_receive.analog_left_Y, 1, (128 - DEAD_ZONE), VELOCITY_LIMIT, 1);
				direction = BWD;			
			}


			if (message_receive.analog_left_Y >= (128 - DEAD_ZONE) && message_receive.analog_left_Y < (128 + DEAD_ZONE))
			{				
				speed_general = 0.0;					
			}
			if (message_receive.analog_left_Y >= (128 + DEAD_ZONE) && message_receive.analog_left_Y <= 255)
			{		
				speed_general = map(message_receive.analog_left_Y - (128 + DEAD_ZONE), 1, (128 - DEAD_ZONE), 1, VELOCITY_LIMIT);
				direction = FWD;			
			}

			// STEERING
			
			if (message_receive.analog_right_X >= 0 && message_receive.analog_right_X < (128 - DEAD_ZONE))
			{
				steer_left = map(message_receive.analog_right_X, 0, 128 - DEAD_ZONE, 0, 100);
				steer_right = 100.0;
			}

			if (message_receive.analog_right_X >= (128 - DEAD_ZONE) && message_receive.analog_right_X < (128 + DEAD_ZONE))
			{
				steer_left = 100.0;
				steer_right = 100.0;
			}

			if (message_receive.analog_right_X >= (128 + DEAD_ZONE) && message_receive.analog_right_X <= 255)
			{
				//steer_right = map(message.analog_right_X, (128 + dead_zone), 255, 100, 0);
				steer_right = float((message_receive.analog_right_X - 145) * (-10) + 100);
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

		case CONTROLS_AUTONOMUS:
			control_mode = CONTROLS_AUTONOMUS;
			if (distance_measured > 60)
			{
				speed_left = 20;
				speed_right = 20;
			}

			if (distance_measured <= 60 && distance_measured > 20)
			{
				speed_left = 10;
				speed_right = 10;
			}

			if (distance_measured <= 20)
			{
				speed_left = 0;
				speed_right = 0;
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
	pid.errorLeft = (int)measured_speed_left - (int)speed_left;
	pid.errorRight = (int)measured_speed_right - (int)speed_right;
	switch (pid.errorLeft)
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

	switch (pid.errorRight)
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
	pid.errorLeft = ((int)speed_left - (int)measured_speed_left);
	pid.errorRight = ((int)speed_right - (int)measured_speed_right);

	pid.integralLeft += (((float)pid.errorLeft + (float)pid.errorLeft_last) / 2)*PID_DT/1000;
	pid.integralRight += (((float)pid.errorRight + (float)pid.errorRight_last) / 2)*PID_DT/1000;
	if (abs(speed_general) < 1.0)
	{
		pid.integralLeft = 0.0;
		pid.integralRight = 0.0;
	}

	if (pid.integralLeft > pid.integralLimit)
		pid.integralLeft = pid.integralLimit;
	if (pid.integralLeft < -pid.integralLimit)
		pid.integralLeft = -pid.integralLimit;

	if (pid.integralRight > pid.integralLimit)
		pid.integralRight = pid.integralLimit;
	if (pid.integralRight < -pid.integralLimit)
		pid.integralRight = -pid.integralLimit;

	pid.differentialLeft = pid.errorLeft - pid.errorLeft_last;
	pid.differentialRight = pid.errorRight - pid.errorRight_last;

	PWM_left_motor = pid.Kp * (pid.errorLeft + ((1 / pid.Ki)*pid.integralLeft ) + (pid.Kd * pid.differentialLeft / PID_DT));
	PWM_right_motor = pid.Kp * (pid.errorRight + ((1 / pid.Ki)*pid.integralRight ) + (pid.Kd *pid.differentialRight / PID_DT));

	if (PWM_left_motor > 255.0)
		PWM_left_motor = 255.0;
	if (PWM_left_motor < 0.0)
		PWM_left_motor = 0.0;

	if (PWM_right_motor > 255.0)
		PWM_right_motor = 255.0;
	if (PWM_right_motor < 0.0)
		PWM_right_motor = 0.0;

	pid.errorLeft_last = pid.errorLeft;
	pid.errorRight_last = pid.errorRight;
}

void controlMotors()
{

	if (false && side_switch == 1 && analog_left_switch == 1 && analog_right_switch == 1 && dipSwitch1.pin1_state == 0)
	{
		controls(CONTROLS_STANDARD);		
	}
	else if (false && side_switch == 1 && analog_right_switch == 0 && analog_right_switch == 0 && dipSwitch1.pin1_state == 0)
	{
		controls(CONTROLS_ENCHANCED);
	}
	else if (false && side_switch == 0 && dipSwitch1.pin1_state == 0)
	{
		controls(CONTROLS_MEASURED);			
		PIDtimer.run();
	}
	else if (true || dipSwitch1.pin1_state == 1)
	{
		controls(CONTROLS_AUTONOMUS);
		PIDtimer.run();
	}
	else
	{
		LeftMotor.stop();
		RightMotor.stop();
		SerialTimer1.run();
	}

	if (messages_lost > 2 || empty_receive_data)
	{
		LeftMotor.stop();
		RightMotor.stop();
	}
}