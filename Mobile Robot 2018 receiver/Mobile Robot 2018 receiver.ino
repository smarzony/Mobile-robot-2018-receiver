#include <SimpleTimer.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//SWITCHES
#define RADIO_PRINT_INCOMING_MESSAGE 1

//CONTROLS
#define CONTROLS_STANDARD 0
#define CONTROLS_ENCHANCED 1
#define CONTROLS_MEASURED 2
#define FWD 0
#define BWD 1


//TIMERS
#define ONE_MS 1
#define CURRENT_MEASURE_PERIOD 10
#define SERIAL_PRINT_PERIOD 1000
#define ANALOG_CALIBRATION_TIME 1500
#define VELOCITY_MEASURING_MINIMAL_PERIOD 2
#define PWM_COMPUTE_PERIOD 150
#define PID_DT 150

//CIRCULAR BUFFERS
#define CURRENT_BUFFER_SIZE 20
#define ANALOG_CALIBRATION 1

//PHYSICAL PINS
#define SHUNT_PIN_PLUS A1
#define SHUNT_PIN_MINUS A2

#define SPEED_SENSOR_LEFT 3
#define SPEED_SENSOR_RIGHT 2


//RADIO CONFIG
#define CE 7
#define CSN 8
#define PRINTING 1
#define NOT_PRINTING 0

//RADIO MESSAGE PARTS
#define	ANALOG_LEFT_Y 0
#define ANALOG_LEFT_X 1
#define ANALOG_RIGHT_Y 2
#define ANALOG_RIGHT_X 3
#define	CONTROL_BYTE 4
#define MESSAGE_NO 5

//RGB DIODE
#define R_LED 34
#define G_LED 32
#define B_LED 33

//LIGHTS
#define LEFT_LIGHT 48
#define RIGHT_LIGHT 49

//SERIAL OUTPUT
#define STANDARD 0
#define SPEED_MONITORING 1

class Motor
{
public:
	Motor(int pA, int pB, int pPWM);
	void forward(short speed);
	void backward(short speed);
	void stop();

private:
	int pinA;
	int pinB;
	int pinPWM;
};

struct radioData {
	byte analog_left_X;
	byte analog_left_Y;
	byte analog_right_X;
	byte analog_right_Y;
	byte led_r;
	byte led_g;
	byte led_b;
	byte steering_wheel;
	byte reserved1;
	byte rotory_encoder;
	byte bit_array;
	byte message_no;
};

struct dipSwitch
{
	byte pin1 = 28;
	bool pin1_state;
	byte pin2 = 29;
	bool pin2_state;
	byte pin3 = 30;
	bool pin3_state;
	byte pin4 = 31;
	bool pin4_state;
};

dipSwitch dipSwitch1;

SimpleTimer SerialTimer;
SimpleTimer SerialTimer1;
SimpleTimer SerialTimerPID;
SimpleTimer SpeedMonitorLeft;
SimpleTimer SpeedMonitorRight;
SimpleTimer dipSwitchRead;
SimpleTimer outputSpeed;
SimpleTimer PIDtimer;


//radio variables
radioData message;
RF24 radio(CE, CSN);
const byte rxAddr[6] = { '1','N','o','d','e','1' };
bool empty_receive_data;
bool chip_connected_last_state;

bool radio_not_availalble_counter;
byte incoming_message[6];
short last_message_no, current_message_no, messages_lost;
bool radio_not_availalble = 1;

Motor RightMotor(41, 40, 45);
Motor LeftMotor(43, 42, 44);

float speed_general;
float speed_left;
float speed_right;
float steer_left, steer_right;
bool direction;
float PWM_left_motor;
float PWM_right_motor;
int errorLeft, errorLeft_last;
int errorRight, errorRight_last;
float integralLeft, integralRight;
float differentialLeft, differentialRight;
float Kp = 1.2, Ki = 0.3, Kd = 100;
float integralLimit = 175.0;
float velocityLimit = 80;


byte speed_left_count;
byte speed_right_count;
byte measured_speed_left;
byte measured_speed_right;
byte dead_zone = 17;

bool side_switch, analogLeft_Switch, analogRightSwitch, rotoryEncoder_switch;


//robot control variables
short cBufanalogRightX[ANALOG_CALIBRATION],
cBufanalogRightY[ANALOG_CALIBRATION],
cBufanalogLeftX[ANALOG_CALIBRATION],
cBufanalogLeftY[ANALOG_CALIBRATION],
analogLeftYrest,
analogLeftXrest,
analogRightYrest,
analogRightXrest;

bool done_calibration = false;

//event time variables
unsigned long long now, last_message_read, last_shunt_measure, last_serial_print;
unsigned long long last_velo_measure_left, last_velo_measure_right;

//current measure
float current_array[CURRENT_BUFFER_SIZE];
float current_shunt;
float average_current;

//SERIAL MODE
byte serial_mode = SPEED_MONITORING;

void setup()
{
	pinMode(R_LED, OUTPUT);
	pinMode(B_LED, OUTPUT);
	pinMode(G_LED, OUTPUT);
	pinMode(LEFT_LIGHT, OUTPUT);
	pinMode(RIGHT_LIGHT, OUTPUT);
	pinMode(SPEED_SENSOR_LEFT, INPUT_PULLUP);
	pinMode(SPEED_SENSOR_RIGHT, INPUT_PULLUP);
	pinMode(dipSwitch1.pin1, INPUT_PULLUP);
	pinMode(dipSwitch1.pin2, INPUT_PULLUP);
	pinMode(dipSwitch1.pin3, INPUT_PULLUP);
	pinMode(dipSwitch1.pin4, INPUT_PULLUP);	

	digitalWrite(G_LED, 1);
	Serial.begin(9600);
	Serial.println("Starting...");
	radioConfig();
	Serial.println("Setup completed");
	delay(100);
	digitalWrite(G_LED, 0);

	attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT), leftSpeedSensorInterrupt, RISING);  // Increase counter A when speed sensor pin goes High
	attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT), rightSpeedSensorInterrupt, RISING);

	SerialTimer.setInterval(500, serialPrintSpeedMonitor);
	SerialTimer1.setInterval(500, serialPrintStandard);
	SerialTimerPID.setInterval(500, SerialPrintPID);

	SpeedMonitorLeft.setInterval(500, countLeftSpeed);
	SpeedMonitorRight.setInterval(500, countRightSpeed);

	dipSwitchRead.setInterval(1000, dipSwitchReadEvent);
	outputSpeed.setInterval(PWM_COMPUTE_PERIOD, computePWM);
	PIDtimer.setInterval(PID_DT, computePID);

}

void loop()
{
	dipSwitchRead.run();

	//dead_zone = (float)message.rotory_encoder;



	SpeedMonitorLeft.run();
	SpeedMonitorRight.run();

	if (radio.isChipConnected() == 1 && chip_connected_last_state == 0)
		radioConfig();

	digitalWrite(R_LED, !radio.isChipConnected());
	digitalWrite(LEFT_LIGHT, bitRead(message.bit_array, 1));
	digitalWrite(RIGHT_LIGHT, bitRead(message.bit_array, 2));

	now = millis();
	readRadio(0, NOT_PRINTING);
	shunt_measure(CURRENT_MEASURE_PERIOD);
	//serialPrint(SERIAL_PRINT_PERIOD, SPEED_MONITORING);

	// STOP MOTORS
	if (messages_lost > 2)
	{
		LeftMotor.stop();
		RightMotor.stop();
	}

	// MOVE MOTORS
	
	if (false && bitRead(message.bit_array, 1) == 1 && bitRead(message.bit_array, 2) == 1)
	{
		controls(CONTROLS_STANDARD);
		SerialTimer1.run();
	}
	else if (side_switch == 1)
	{
		digitalWrite(RIGHT_LIGHT, 0);
		controls(CONTROLS_ENCHANCED);
		SerialTimer1.run();
	}
	else if (side_switch == 0)
	{ 
		digitalWrite(RIGHT_LIGHT, 1);
		controls(CONTROLS_MEASURED);
		SerialTimerPID.run();
		PIDtimer.run();
	}
	else
	{
		LeftMotor.stop();
		RightMotor.stop();
		SerialTimer1.run();
	}

	if (message.analog_left_Y == 0 && message.analog_left_X && message.analog_right_X == 0 & message.analog_right_Y == 0)
		empty_receive_data = 1;
	else
		empty_receive_data = 0;

	if (empty_receive_data == 1)
		digitalWrite(R_LED, 1);
	else
		digitalWrite(R_LED, 0);


	// -------------- END OPERATIONS ---------------------
	chip_connected_last_state = radio.isChipConnected();
}

void radioConfig()
{
	/*
	digitalWrite(CE, 0);
	digitalWrite(CSN, 0);
	delay(50);
	*/
	radio.begin();
	radio.setDataRate(RF24_1MBPS);
	radio.setChannel(0);//100	
	// ------------   JAK SIÊ ROZJEBIE TO ZMIEN KANA£ -----
	radio.openReadingPipe(0, rxAddr);
	radio.startListening();
}

void analogCalibration(unsigned long long now, int time, short &leftYrestpos, short &leftXrestpos, short &rightYrestpos, short &rightXrestpos, bool &printed)
{
	if (time > now)
	{
		CircBuffer(cBufanalogLeftY, incoming_message[ANALOG_LEFT_Y], sizeof(cBufanalogLeftY));
		CircBuffer(cBufanalogLeftX, incoming_message[ANALOG_LEFT_X], sizeof(cBufanalogLeftX));
		CircBuffer(cBufanalogRightY, incoming_message[ANALOG_RIGHT_Y], sizeof(cBufanalogRightY));
		CircBuffer(cBufanalogRightX, incoming_message[ANALOG_RIGHT_X], sizeof(cBufanalogRightX));
	}
	if (time < now && printed == false)
	{
		short sum;
		for (int i = 0; i < sizeof(cBufanalogLeftY) - 1; i++)
		{
			sum = sum + cBufanalogLeftY[i];
		}
		leftYrestpos = sum / sizeof(cBufanalogLeftY);
		sum = 0;

		for (int i = 0; i < sizeof(cBufanalogLeftX) - 1; i++)
		{
			sum = sum + cBufanalogLeftX[i];
		}
		leftXrestpos = sum / sizeof(cBufanalogLeftX);
		sum = 0;

		for (int i = 0; i < sizeof(cBufanalogRightY) - 1; i++)
		{
			sum = sum + cBufanalogRightY[i];
		}
		rightYrestpos = sum / sizeof(cBufanalogRightY);
		sum = 0;

		for (int i = 0; i < sizeof(cBufanalogRightX) - 1; i++)
		{
			sum = sum + cBufanalogRightX[i];
		}
		rightXrestpos = sum / sizeof(cBufanalogRightX);
		sum = 0;

		Serial.print("Calibration done: ");
		Serial.print(leftYrestpos);
		Serial.print(' ');
		Serial.print(leftXrestpos);
		Serial.print(' ');
		Serial.print(rightYrestpos);
		Serial.print(' ');
		Serial.print(rightXrestpos);
		Serial.print('\n');

		printed = true;
	}
}

void SerialDummy()
{
	Serial.print("Uptime (s): ");
	Serial.println(millis() / 1000);
}

void leftSpeedSensorInterrupt()
{
	if (now - last_velo_measure_left > VELOCITY_MEASURING_MINIMAL_PERIOD)
	{
		last_velo_measure_left = now;
		speed_left_count++;
	}
	
}

void rightSpeedSensorInterrupt()
{		
	if (now - last_velo_measure_right > VELOCITY_MEASURING_MINIMAL_PERIOD)
	{
		last_velo_measure_right = now;
		speed_right_count++;
	}
}

void countLeftSpeed()
{
	measured_speed_left = speed_left_count;
	speed_left_count = 0;
}

void countRightSpeed()
{
	measured_speed_right = speed_right_count;
	speed_right_count = 0;
}

void dipSwitchReadEvent()
{
	dipSwitch1.pin1_state = digitalRead(dipSwitch1.pin1);
	dipSwitch1.pin2_state = digitalRead(dipSwitch1.pin2);
	dipSwitch1.pin3_state = digitalRead(dipSwitch1.pin3);
	dipSwitch1.pin4_state = digitalRead(dipSwitch1.pin4);
}