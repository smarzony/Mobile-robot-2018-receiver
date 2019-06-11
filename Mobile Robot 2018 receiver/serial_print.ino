
void serialPrintStandard()
{

	String output;
	output += message.analog_left_X;
	output += ' ';
	output += message.analog_left_Y;
	output += ' ';
	output += message.analog_right_X;
	output += ' ';
	output += message.analog_right_Y;
	output += ' ';
	output += message.message_no;
	output += ' ';
	output += message.rotory_encoder;

	Serial.println(output);

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
	output += errorLeft;
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
	*/
	output += "DZ = ";
	output += dead_zone;
	output += "\tvelo = [";
	output += (byte)speed_general;
	output += ", ";
	output += measured_speed_left;
	output += ", ";
	output += measured_speed_right;
	output += "]\terror = [";
	output += errorLeft;
	output += ", ";
	output += errorRight;
	output += "]\tI =[";
	output += (byte)integralLeft;
	output += ", ";
	output += (byte)integralRight;
	output += "]\td = [";
	output += (byte)differentialLeft;
	output += ", ";
	output += (byte)differentialRight;
	output += "]\tPWM = [";
	output += (byte)PWM_left_motor;
	output += ", ";
	output += (byte)PWM_right_motor;
	output += "]";


	Serial.println(output);
}
