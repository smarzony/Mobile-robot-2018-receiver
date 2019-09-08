#include <NewPing.h>
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
#define CONTROLS_AUTONOMUS 3
#define FWD 0
#define BWD 1
#define DEAD_ZONE 17
#define VELOCITY_LIMIT 80.0

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
#define TRIGGER_PIN 14
#define ECHO_PIN 15

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
	int pinA,
		pinB,
		pinPWM;
};

struct PIDstruct {
	int errorLeft, 
		errorLeft_last,
		errorRight, 
		errorRight_last;

	float integralLeft, 
		integralRight,
		differentialLeft, 
		differentialRight,
		Kp = 1.2,
		Ki = 0.3,
		Kd = 100,
		integralLimit = 175.0;
};

struct radioDataReceive {
	byte analog_left_X,
		analog_left_Y,
		analog_right_X,
		analog_right_Y,
		led_r,
		led_g,
		led_b,
		steering_wheel,
		reserved1,
		rotory_encoder,
		bit_array,
		message_no;
}; 

struct radioDataTransmit {
	byte velocity_measured_left,
		velocity_measured_right,
		distance,
		control_mode,
		reserved4,
		reserved5,
		reserved6,
		reserved7,
		reserved8,
		message_no;
};

struct dipSwitch
{
	byte pin1 = 26;	
	byte pin2 = 27;	
	byte pin3 = 28;	
	byte pin4 = 29;

	bool pin1_state,
		pin2_state,
		pin3_state,
		pin4_state;
};

dipSwitch dipSwitch1;

SimpleTimer SerialTimer,
			SerialTimer1,
			SerialTimerPID,
			SerialTimerSend,
			SpeedMonitorLeft,
			SpeedMonitorRight,
			dipSwitchRead,
			outputSpeed,
			PIDtimer,
			SendRadioTimer,
			CheckDistanceTimer,
			SerialTimerAutonomusMode;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);


//radio variables
radioDataReceive message_receive;
radioDataTransmit message_transmit;

RF24 radio(CE, CSN);
const byte rxAddr[6] = { '1','N','o','d','e','1' };
const byte txAddr[6] = { '1','N','o','d','e','2' };
bool empty_receive_data;
bool chip_connected_last_state;

bool radio_not_availalble_counter;
byte incoming_message[6];
short last_message_no, current_message_no, messages_lost;
bool radio_not_availalble = 1;

Motor RightMotor(41, 40, 45);
Motor LeftMotor(43, 42, 44);

PIDstruct pid;

float speed_general;
float speed_left;
float speed_right;
float steer_left, steer_right;
float PWM_left_motor;
float PWM_right_motor;

bool direction;

/*
int errorLeft, errorLeft_last;
int errorRight, errorRight_last;
float integralLeft, integralRight;
float differentialLeft, differentialRight;
float Kp = 1.2, Ki = 0.3, Kd = 100;
float integralLimit = 175.0;
*/


byte speed_left_count;
byte speed_right_count;
byte measured_speed_left;
byte measured_speed_right;
byte control_mode;


bool side_switch, 
	analog_left_switch, 
	analog_right_switch, 
	rotory_encoder_switch;


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
unsigned long long now, 
	last_message_read, 
	last_shunt_measure, 
	last_serial_print, 
	last_velo_measure_left, 
	last_velo_measure_right;


//current measure
float current_array[CURRENT_BUFFER_SIZE];
float current_shunt;
float average_current;

//distance measure
byte distance_measured;

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
	
	Serial.begin(9600);
	Serial.println("Starting...");
	radioConfig();
	Serial.println("Setup completed");
	delay(100);
	

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
	SendRadioTimer.setInterval(500, sendRadio);
	CheckDistanceTimer.setInterval(250, CheckDistance);
	SerialTimerSend.setInterval(500, SerialPrintSendMSG);
	SerialTimerAutonomusMode.setInterval(500, SerialPrintControlsAutonomus);
}

void loop()
{
	// TIMERS
	now = millis();
	dipSwitchRead.run();
	SpeedMonitorLeft.run();
	SpeedMonitorRight.run();
	//SendRadioTimer.run();
	CheckDistanceTimer.run();
	if (radio.isChipConnected() == 1 && chip_connected_last_state == 0)
		radioConfig();

	// TESTS
	/*analog_control_step = message_receive.rotory_encoder;
	if (analog_control_step > 200)
		analog_control_step = 200;
		*/

	// OUTPUTS
	digitalWrite(LEFT_LIGHT, analog_left_switch);
	digitalWrite(RIGHT_LIGHT, analog_right_switch);

	if (empty_receive_data == 1)
		digitalWrite(R_LED, 1);
	else
		digitalWrite(R_LED, 0);


	// COMMUNICATION
	readRadio(0, NOT_PRINTING);

	// MEASURES
	shunt_measure(CURRENT_MEASURE_PERIOD);

	// MOVE MOTORS
	controlMotors();

	if (speed_left > 0 || speed_right > 0)
	{
		switch (control_mode)
		{
		case CONTROLS_STANDARD:
			SerialTimer1.run();
			break;
		case CONTROLS_ENCHANCED:
			SerialTimer1.run();
			break;
		case CONTROLS_MEASURED:
			SerialTimerPID.run();
			break;
		case CONTROLS_AUTONOMUS:
			SerialTimerAutonomusMode.run();
			break;
		}
	}
	else
	{
		if (control_mode != CONTROLS_AUTONOMUS)
		{
			//SerialTimerSend.run();
			SerialTimerAutonomusMode.run();
		}
		else
			SerialTimerAutonomusMode.run();
	}


	// -------------- END OPERATIONS ---------------------
	chip_connected_last_state = radio.isChipConnected();
}



/*
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
*/

/*
void SerialDummy()
{
	Serial.print("Uptime (s): ");
	Serial.println(millis() / 1000);
}
*/



void dipSwitchReadEvent()
{
	dipSwitch1.pin1_state = digitalRead(dipSwitch1.pin1);
	dipSwitch1.pin2_state = digitalRead(dipSwitch1.pin2);
	dipSwitch1.pin3_state = digitalRead(dipSwitch1.pin3);
	dipSwitch1.pin4_state = digitalRead(dipSwitch1.pin4);
}

void CheckDistance()
{
	distance_measured = sonar.ping_cm();
}