#include <NewPing.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//SWITCHES
#define RADIO_PRINT_INCOMING_MESSAGE 1

//CONTROLS
#define CONTROLS_NONE 0
#define CONTROLS_STANDARD 1
#define CONTROLS_ENCHANCED 2
#define CONTROLS_MEASURED 3
#define CONTROLS_AUTONOMUS 4

#define FWD 0
#define BWD 1
#define DEAD_ZONE 40

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
#define BUZZER_PIN 4
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
	void forward(short speed, bool interlock);
	void backward(short speed, bool interlock);
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
		Kp = 1.3,
		Ki = 0.4,
		Kd = 100,
		integralLimit = 175.0;
};

struct radioDataReceive {
	byte analog_left_X,
		analog_left_Y,
		analog_right_X,
		analog_right_Y,
		servo_0,
		led_g,
		led_b,
		potentiometer,
		control_mode,
		rotory_encoder,
		bit_array,
		message_no;
}; 

struct radioDataTransmit {
	byte velocity_measured_left,
		velocity_measured_right,
		distance,
		control_mode,
		time_delay,
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

struct limitSwitch
{
	byte pin_left = 17;
	byte pin_right = 16;
	byte pin_spare00 = 0;
	byte pin_spare01 = 0;

	bool left,
		right,
		spare00,
		spare01;
};

limitSwitch limitSwitches, limitswitches_last;

dipSwitch dipSwitch1;

unsigned long long SerialTimer,
			SerialTimer1,
			SerialTimerPID = 7,
			SerialTimerSend,
			SpeedMonitorLeft = 61,
			SpeedMonitorRight = 67,
			dipSwitchRead = 23,
			PIDtimer = 43,
			SendRadioTimer =12,
			CheckDistanceTimer = 37,
			SerialTimerAutonomusMode, 
			SerialPrintLimitSwitches,
			RadioTimeoutTimer,
			AliveTimer,
			PlayToneOKTimer,
			PlayToneNGTimer;			

unsigned long long  Autonomous_wait_to_go_bwd;

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
float velocity_limit;

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

bool analog_left_switch_last,
	analog_right_switch_last;

bool play_tone_OK,
	play_tone_NG;


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
	last_velo_measure_right,
	now_micros;


//current measure
float current_array[CURRENT_BUFFER_SIZE];
float current_shunt;
float average_current;

//distance measure
byte distance_measured, distance_measured_last;

//SERIAL MODE
byte serial_mode = SPEED_MONITORING;

// RGB LED
bool led_state_green;



void setup()
{
	pinMode(R_LED, OUTPUT);
	pinMode(B_LED, OUTPUT);
	pinMode(G_LED, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(LEFT_LIGHT, OUTPUT);
	pinMode(RIGHT_LIGHT, OUTPUT);
	pinMode(SPEED_SENSOR_LEFT, INPUT_PULLUP);
	pinMode(SPEED_SENSOR_RIGHT, INPUT_PULLUP);
	pinMode(dipSwitch1.pin1, INPUT_PULLUP);
	pinMode(dipSwitch1.pin2, INPUT_PULLUP);
	pinMode(dipSwitch1.pin3, INPUT_PULLUP);
	pinMode(dipSwitch1.pin4, INPUT_PULLUP);	
	pinMode(limitSwitches.pin_left, INPUT_PULLUP);
	pinMode(limitSwitches.pin_right, INPUT_PULLUP);
	
	attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT), leftSpeedSensorInterrupt, RISING);  // Increase counter A when speed sensor pin goes High
	attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT), rightSpeedSensorInterrupt, RISING);

	Serial.begin(9600);
	Serial.println("Starting...");
	radioConfig();
	Serial.println("Setup completed");
	//delay(100);
}

void loop()
{
	// TIMERS
	now = millis();
	if (now - AliveTimer > 1000)
	{
		AliveTimer = now;
		led_state_green = !led_state_green;
	}

	if (now - SerialTimer > 250)
	{
		SerialTimer = now;
		serialPrintStandard();
	}

	if (now - RadioTimeoutTimer > 1000)
			digitalWrite(R_LED, 1);
		else
			digitalWrite(R_LED, 0);	

	if (now - RadioTimeoutTimer > 3000)
	{
		radioConfig();
		RadioTimeoutTimer = now;
	}


	if (now - dipSwitchRead > 1000)
	{
		dipSwitchRead = now;
		dipSwitchReadEvent();
	}

	if (now - SpeedMonitorLeft > 500)
	{
		SpeedMonitorLeft = now;
		countLeftSpeed();
	}

	if (now - SpeedMonitorRight > 500)
	{
		SpeedMonitorRight = now;
		countRightSpeed();
	}

	if (now - SendRadioTimer > 500)
	{
		SendRadioTimer = now;
		sendRadio();
	}
	if (now - CheckDistanceTimer > 75)
	{
		CheckDistanceTimer = now;
		CheckDistance();
	}

	/*
	if (now - SerialPrintLimitSwitches > 500)
	{
		SerialPrintLimitSwitches = now;
		Serial.print("Limit switches: ");
		Serial.print(limitSwitches.left);
		Serial.print(" ");
		Serial.println(limitSwitches.right);
	}
	*/

	/*
	if (radio.isChipConnected() == 1 && chip_connected_last_state == 0)
		radioConfig();
	*/

	// TESTS
	/*analog_control_step = message_receive.rotory_encoder;
	if (analog_control_step > 200)
		analog_control_step = 200;
		*/

	// INPUTS
	limitSwitches.left = !digitalRead(limitSwitches.pin_left);
	limitSwitches.right = !digitalRead(limitSwitches.pin_right);

	if (analog_left_switch_last != analog_left_switch && analog_left_switch == true)
	{
		play_tone_OK = true;
		PlayToneOKTimer = now;
	}

	if (play_tone_OK)
	{
		if (now - PlayToneOKTimer < 250)
		{
			//tone(BUZZER_PIN, 440);
			digitalWrite(BUZZER_PIN, HIGH);
		}
		if (now - PlayToneOKTimer >= 250 && now - PlayToneOKTimer < 500)
		{
			//tone(BUZZER_PIN, 523);
			digitalWrite(BUZZER_PIN, LOW);
		}
		if (now - PlayToneOKTimer >= 500 && now - PlayToneOKTimer < 750)
		{
			//tone(BUZZER_PIN, 440);
			digitalWrite(BUZZER_PIN, HIGH);
		}
		if (now - PlayToneOKTimer >= 750)
		{
			//noTone(BUZZER_PIN);
			digitalWrite(BUZZER_PIN, LOW);
			play_tone_OK = false;
		}		
	}


	// OUTPUTS

	digitalWrite(LEFT_LIGHT, analog_left_switch);
	digitalWrite(RIGHT_LIGHT, analog_right_switch);

	digitalWrite(G_LED, led_state_green);

	// COMMUNICATION
	readRadio(0, NOT_PRINTING);

	// MEASURES
	//shunt_measure(CURRENT_MEASURE_PERIOD);

	// MOVE MOTORS
	controlMotors();

	/*
	if (speed_left > 0 || speed_right > 0)
	{
		switch (control_mode)
		{
		case CONTROLS_STANDARD:
			if (now - SerialTimer1 > 500)
			{
				SerialTimer1 = now;
				//serialPrintStandard();
			}
			break;
		case CONTROLS_ENCHANCED:
			if (now - SerialTimer1 > 500)
			{
				SerialTimer1 = now;
				//serialPrintStandard();
			}
			break;
		case CONTROLS_MEASURED:
			if (now - SerialTimerPID > 500)
			{
				SerialTimerPID = now;
				//SerialPrintPID();
			}
			break;
		case CONTROLS_AUTONOMUS:
			if (now - SerialTimerAutonomusMode > 500)
			{
				SerialTimerAutonomusMode = now;
				//SerialPrintControlsAutonomus();
			}
			break;
		}
	}
	else
	{
		if (control_mode != CONTROLS_AUTONOMUS)
		{
			if (now - SerialTimerAutonomusMode > 500)
			{
				SerialTimerAutonomusMode = now;
				//SerialPrintControlsAutonomus();
			}			
		}
		else
			if (now - SerialTimerAutonomusMode > 500)
			{
				SerialTimerAutonomusMode = now;
				//SerialPrintControlsAutonomus();
			}
	}
	*/

	//SERIAL DEBUG
	//Serial.println(byte((now - RadioTimeoutTimer) ));

	//WATCHDOGS


	// -------------- END OPERATIONS ---------------------
	//chip_connected_last_state = radio.isChipConnected();
	limitswitches_last = limitSwitches;
	analog_left_switch_last = analog_left_switch;
	analog_right_switch_last = analog_right_switch;
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
	int raw_data = sonar.ping_cm();
	if (raw_data > 0)
	{
		distance_measured_last = distance_measured;
		distance_measured = raw_data;
	}	
}