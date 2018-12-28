

Motor::Motor(int pA, int pB, int pPWM)
{
	Motor::pinA = pA;
	Motor::pinB = pB;
	Motor::pinPWM = pPWM;

	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
}

void Motor::forward(short speed)
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 1);
	analogWrite(pinPWM, speed);
}
void Motor::backward(short speed)
{
	digitalWrite(pinA, 1);
	digitalWrite(pinB, 0);
	analogWrite(pinPWM, speed);
}
void Motor::stop()
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 0);
	analogWrite(pinPWM, 0);
}


