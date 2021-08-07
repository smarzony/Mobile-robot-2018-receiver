

Motor::Motor(int pA, int pB, int pPWM)
{
	Motor::pinA = pA;
	Motor::pinB = pB;
	Motor::pinPWM = pPWM;

	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
}

void Motor::forward(short speed, bool interlock)
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 1);
	if (interlock)
		analogWrite(pinPWM, speed);
	else
		analogWrite(pinPWM, 0);
}
void Motor::backward(short speed, bool interlock)
{
	digitalWrite(pinA, 1);
	digitalWrite(pinB, 0);
	if (interlock)
		analogWrite(pinPWM, speed);
	else
		analogWrite(pinPWM, 0);
}
void Motor::stop()
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 0);
	analogWrite(pinPWM, 0);
}


