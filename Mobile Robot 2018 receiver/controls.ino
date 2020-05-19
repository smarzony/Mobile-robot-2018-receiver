uint8_t distance_limit()
{
	if (measured_speed_left < 10 || measured_speed_right < 10)
		return 10;
	else if (measured_speed_left >= 10 && measured_speed_right >= 10)
		return 30;
	else if (measured_speed_left >= 20 && measured_speed_right >= 20)
		return 40;
}

void controls(byte control_type)
{
	uint8_t limit = distance_limit();
	if (message_receive.message_no != 0 && !empty_receive_data)
	{
		float set_velo, set_velo_left, set_velo_right;
		float steering_left, steering_right, steering;
		switch (control_type)
		{
		case CONTROLS_NONE:
			control_mode = CONTROLS_NONE;
			break;

		case CONTROLS_STANDARD:
			control_mode = CONTROLS_STANDARD;
			
			switch (message_receive.analog_left_Y)
			{
			case 0 ... (127 - DEAD_ZONE):
				set_velo = map(message_receive.analog_left_Y, 127 - DEAD_ZONE, 0, 0, velocity_limit);
				LeftMotor.backward(set_velo, 1);
				break;

			case (127 - DEAD_ZONE + 1) ... (127 + DEAD_ZONE):
				LeftMotor.stop();
				break;
			case (127 + DEAD_ZONE + 1) ... 255:
				set_velo = map(message_receive.analog_left_Y, 127 + DEAD_ZONE + 1, 255, 0, velocity_limit);
				LeftMotor.forward(velocity_limit, !limitSwitches.left && !limitSwitches.right && (distance_measured > limit));
				break;
			}

			switch (message_receive.analog_right_Y)
			{
			case 0 ... (127 - DEAD_ZONE) :
				set_velo = map(message_receive.analog_right_Y, 127 - DEAD_ZONE, 0, 0, velocity_limit);
				RightMotor.backward(set_velo, 1);
				break;
			case (127 - DEAD_ZONE + 1) ... (127 + DEAD_ZONE) :
				RightMotor.stop();
				break;
			case (127 + DEAD_ZONE + 1) ... 255:
				set_velo = map(message_receive.analog_right_Y, 127 + DEAD_ZONE + 1, 255, 0, velocity_limit);
				RightMotor.forward(set_velo, !limitSwitches.left && !limitSwitches.right && (distance_measured > limit));
				break;
			}
			break;

		case CONTROLS_ENCHANCED:
			control_mode = CONTROLS_ENCHANCED;
			switch (message_receive.analog_left_Y)
			{
				case 0 ... (127 - DEAD_ZONE) :
					set_velo = map(message_receive.analog_left_Y, 127 - DEAD_ZONE, 0, 0, velocity_limit);
					direction = BWD;
					break;

				case (127 - DEAD_ZONE + 1) ... (127 + DEAD_ZONE) :
					LeftMotor.stop();
					RightMotor.stop();
					direction = FWD;
					break;
				case (127 + DEAD_ZONE + 1) ... 255:
					set_velo = map(message_receive.analog_left_Y, 127 + DEAD_ZONE + 1, 255, 0, velocity_limit);
					direction = FWD;
					break;
			}

			switch (message_receive.analog_right_X)
			{
				case 0 ... (127 - DEAD_ZONE) :
					steering_left = map(message_receive.analog_right_X, 127 - DEAD_ZONE, 0, 100, 1);
					steering_right = 100.0;
					break;

				case (127 - DEAD_ZONE + 1) ... (127 + DEAD_ZONE) :
					steering_left = 100.0;
					steering_right = 100.0;
					break;
				case (127 + DEAD_ZONE + 1) ... 255:
					steering_left = 100.0;
					steering_right = map(message_receive.analog_right_X, 127 + DEAD_ZONE + 1, 255, 100, 1);
					break;
			}

			set_velo_left = set_velo * (abs(steering_left) / 100);
			set_velo_right = set_velo * (abs(steering_right) / 100);
			if (direction == BWD)
			{
				LeftMotor.backward(set_velo_left, 1);
				RightMotor.backward(set_velo_right, 1);
			}
			else if (direction == FWD)
			{
				LeftMotor.forward(set_velo_left, !limitSwitches.left && !limitSwitches.right && (distance_measured > limit));
				RightMotor.forward(set_velo_right, !limitSwitches.left && !limitSwitches.right && (distance_measured > limit));
			}
			
			break;

		case CONTROLS_MEASURED:
			control_mode = CONTROLS_MEASURED;
			// SPEED
			if (message_receive.analog_left_Y >= 0 && message_receive.analog_left_Y < (128 - DEAD_ZONE))
			{
				speed_general = map(message_receive.analog_left_Y, 1, (128 - DEAD_ZONE), velocity_limit, 1);
				direction = BWD;			
			}


			if (message_receive.analog_left_Y >= (128 - DEAD_ZONE) && message_receive.analog_left_Y < (128 + DEAD_ZONE))
			{				
				speed_general = 0.0;					
			}
			if (message_receive.analog_left_Y >= (128 + DEAD_ZONE) && message_receive.analog_left_Y <= 255)
			{		
				speed_general = map(message_receive.analog_left_Y - (128 + DEAD_ZONE), 1, (128 - DEAD_ZONE), 1, velocity_limit);
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
				LeftMotor.forward((short)PWM_left_motor, !limitSwitches.left && !limitSwitches.right);
				RightMotor.forward((short)PWM_right_motor, !limitSwitches.left && !limitSwitches.right);
			}
			else if (direction == BWD)
			{
				LeftMotor.backward((short)PWM_left_motor, 1);
				RightMotor.backward((short)PWM_right_motor, 1);
			}
			break;

		case CONTROLS_AUTONOMUS:
			control_mode = CONTROLS_AUTONOMUS;
			if (distance_measured > 60)
			{
				/*speed_left = 100;				
				speed_right = 100;*/
				speed_left = message_receive.potentiometer;
				speed_right = message_receive.potentiometer;
				LeftMotor.forward((short)PWM_left_motor, !limitSwitches.left && !limitSwitches.right);
				RightMotor.forward((short)PWM_right_motor, !limitSwitches.left && !limitSwitches.right);
			}

			if (distance_measured <= 60 && distance_measured > 20)
			{
				/*speed_left = 80;
				speed_right = 80;*/
				speed_left = message_receive.potentiometer/2;
				speed_right = message_receive.potentiometer/2;

				LeftMotor.forward((short)PWM_left_motor, !limitSwitches.left && !limitSwitches.right);
				RightMotor.forward((short)PWM_right_motor, !limitSwitches.left && !limitSwitches.right);
			}

			if (distance_measured_last > 20 && distance_measured <= 20)
				Autonomous_wait_to_go_bwd = now;

			if (distance_measured <= 20)
			{

				/*speed_left = 80;
				speed_right = 80;*/
				speed_left = message_receive.potentiometer / 2;
				speed_right = message_receive.potentiometer / 2;
				
				if (now - Autonomous_wait_to_go_bwd > 5000)
				{
					LeftMotor.backward((short)PWM_left_motor, 1);
					RightMotor.backward((short)PWM_right_motor, 1);
				}
				else
				{
					LeftMotor.forward((short)PWM_left_motor, !limitSwitches.left && !limitSwitches.right);
					RightMotor.backward((short)PWM_right_motor, 1);
				}
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

	PWM_left_motor = pid.Kp*1.2 * (pid.errorLeft + ((1 / pid.Ki)*pid.integralLeft ) + (pid.Kd * pid.differentialLeft / PID_DT));
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
	if (message_receive.control_mode == CONTROLS_NONE)
		controls(CONTROLS_NONE);
	else if (message_receive.control_mode == CONTROLS_STANDARD)
		controls(CONTROLS_STANDARD);
	else if (message_receive.control_mode == CONTROLS_ENCHANCED)
		controls(CONTROLS_ENCHANCED);
	else if (message_receive.control_mode == CONTROLS_MEASURED)
		controls(CONTROLS_MEASURED);
	else if (message_receive.control_mode == CONTROLS_AUTONOMUS)
		controls(CONTROLS_AUTONOMUS);

	if (control_mode == CONTROLS_MEASURED || control_mode == CONTROLS_AUTONOMUS)
		if (now - PIDtimer > PID_DT)
		{
			PIDtimer = now;
			computePID();
		}

	if (messages_lost > 2 || empty_receive_data)
	{
		LeftMotor.stop();
		RightMotor.stop();
	}
}