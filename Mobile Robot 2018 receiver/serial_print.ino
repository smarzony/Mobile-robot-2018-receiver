
void serialPrintStandard()
{

	String output = "[XL YL XR YR MSG CTRL SS] = ";
	output += message_receive.analog_left_X;
	output += ' ';
	output += message_receive.analog_left_Y;
	output += ' ';
	output += message_receive.analog_right_X;
	output += ' ';
	output += message_receive.analog_right_Y;
	output += ' ';
	output += message_receive.message_no;
	output += ' ';
	output += message_receive.control_mode;
	output += ' ';
	output += side_switch;

	Serial.println(output);

}
void serialPrintSpeedMonitor()
{
	String output;
	output = "L_Y:";
	output += message_receive.analog_left_Y;
	output += " R_X:";
	output += message_receive.analog_right_X;

	if (direction == FWD)
		output += " FWD";
	else if (direction == BWD)
		output += " BWD";

	output += " S_G:";
	output += speed_general;
	output += " S_L:";
	output += speed_left;
	output += " S_R:";
	/*
	output += speed_right;
	output += " St_L:";
	output += steer_left;
	output += " St_R:";
	output += steer_right;
	*/
	output += " S_L_M:";
	output += measured_speed_left;
	/*
	output += " S_R_M:";	
	output += measured_speed_right;
	*/
	output += " E_L:";	
	output += pid.errorLeft;
	output += " PWM_L";
	output += PWM_left_motor;

	Serial.println(output);
}

void SerialPrintPID()
{
	String output;
	/*
	output += "[Kp, Ki, Kd] = [";
	output += Kp;
	output += ", ";
	output += Ki;
	output += ", ";
	output += Kd;
	output += "]\t";	
	output += "DZ = ";
	output += DEAD_ZONE\t;
	*/
	output += "velo = [";
	output += (byte)speed_general;
	output += ", ";
	output += measured_speed_left;
	output += ", ";
	output += measured_speed_right;
	output += "]\terror = [";
	output += pid.errorLeft;
	output += ", ";
	output += pid.errorRight;
	output += "]\tI =[";
	output += (byte)pid.integralLeft;
	output += ", ";
	output += (byte)pid.integralRight;
	output += "]\td = [";
	output += (byte)pid.differentialLeft;
	output += ", ";
	output += (byte)pid.differentialRight;
	output += "]\tPWM = [";
	output += (byte)PWM_left_motor;
	output += ", ";
	output += (byte)PWM_right_motor;
	output += "]";


	Serial.println(output);
}

void SerialPrintSendMSG()
{
	String output;
	output += "No : ";
	output += message_transmit.message_no;
	output += ", Dist : ";
	output += message_transmit.distance;
	output += ", Mode : ";
	output += message_transmit.control_mode;
	output += ", velo = [";
	output += message_transmit.velocity_measured_left;
	output += ", ";
	output += message_transmit.velocity_measured_right;
	output += "]";


	Serial.println(output);
}

void SerialPrintControlsAutonomus()
{
	String output;
	output += "Dist : ";
	output += distance_measured;
	output += ", velo_in = [";
	output += (int)speed_left;
	output += ", ";
	output += (int)speed_right;
	output += "]";
	output += ", velo_out = [";
	output += (int)measured_speed_left;
	output += ", ";
	output += (int)measured_speed_right;
	output += "]";
	Serial.println(output);
}
