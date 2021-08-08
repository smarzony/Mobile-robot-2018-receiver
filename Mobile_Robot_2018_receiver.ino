#include <NewPing.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>

//SWITCHES
#define RADIO_PRINT_INCOMING_MESSAGE 1

//CONTROLS
#define CONTROLS_NONE 0
#define CONTROLS_TANK 1
#define CONTROLS_CAR 2
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
#define LIMIT_SWITCH_LEFT 16
#define LIMIT_SWITCH_RIGHT 17
#define BUZZER_PIN 4
#define SPEED_SENSOR_LEFT 3
#define SPEED_SENSOR_RIGHT 2
#define FRONT_LIGHTS_PIN 38
#define CAMERA_ENABLE_PIN 39

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

struct machineState {
    float velocity_limit;
    uint8_t empty_receive_data;
    uint8_t empty_messages;
    uint8_t radio_print_counter;
    uint8_t speed;
    uint8_t speed_direction;
    uint8_t steering;
    uint8_t steering_direction;
	uint16_t azimuth;
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
  uint8_t analog_left_X,
       analog_left_Y,
       analog_right_X,
       analog_right_Y,
       reserved0,
       reserved1,
       reserved2,
       potentiometer,
       control_mode,
       reserved3,
       bit_array,
       message_no;
};

struct radioDataTransmit {
  uint8_t velocity_measured_left,
       velocity_measured_right,
       distance,
       control_mode,
       time_delay,
       azimuth1,
       azimuth2,
       reserved7,
       reserved8,
       message_no;
};

struct dipSwitch
{
	uint8_t pin1 = 26;	
	uint8_t pin2 = 27;	
	uint8_t pin3 = 28;	
	uint8_t pin4 = 29;

	bool pin1_state,
		pin2_state,
		pin3_state,
		pin4_state;
};

struct limitSwitch
{
	uint8_t pin_left = LIMIT_SWITCH_LEFT;
	uint8_t pin_right = LIMIT_SWITCH_RIGHT;
	uint8_t pin_spare00 = 0;
	uint8_t pin_spare01 = 0;

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
			PlayToneNGTimer,
			CameraOnTimer;


unsigned long long  Autonomous_wait_to_go_bwd;

TinyGPSPlus gps;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);
QMC5883LCompass compass;

machineState machine_state;
//radio variables
radioDataReceive message_receive;
radioDataTransmit message_transmit;

RF24 radio(CE, CSN);
const uint8_t rxAddr[6] = { '1','N','o','d','e','1' };
const uint8_t txAddr[6] = { '1','N','o','d','e','2' };
bool empty_receive_data;
bool chip_connected_last_state;

uint8_t radio_not_availalble_counter;
uint8_t incoming_message[6];
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


uint8_t speed_left_count;
uint8_t speed_right_count;
uint8_t measured_speed_left;
uint8_t measured_speed_right;
uint8_t control_mode;


bool side_switch, 
	analog_left_switch, 
	analog_right_switch, 
	front_lights_switch,
	camera_enable_switch;

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
	last_azimuth_timer,
	now_micros;


//current measure
float current_array[CURRENT_BUFFER_SIZE];
float current_shunt;
float average_current;

//distance measure
uint8_t distance_measured, distance_measured_last;

//SERIAL MODE
uint8_t serial_mode = SPEED_MONITORING;

// RGB LED
bool led_state_green;



void setup()
{
	pinMode(R_LED, OUTPUT);
	pinMode(B_LED, OUTPUT);
	pinMode(G_LED, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(FRONT_LIGHTS_PIN, OUTPUT);
	pinMode(CAMERA_ENABLE_PIN, OUTPUT);

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

	Serial.begin(115200);
	Serial2.begin(9600);
	Serial.println("Starting...");
	radioConfig();
	compass.init();
  	compass.setCalibration(-1551, 1353, -1982, 552, -1320, 1531);
	Serial.println("Setup completed");
	//delay(100);
}

void loop()
{
	bool analog_left_switch_buffer = analog_left_switch;
	bool analog_right_switch_buffer = analog_right_switch;

	if (Serial2.available() > 0)
	{
		if (gps.encode(Serial2.read()))
		displayInfo();
	}


	// TIMERS
	now = millis();
	if (now - last_azimuth_timer > 50)
	{
		// Read compass values
		compass.read();

		// Return Azimuth reading
		machine_state.azimuth = compass.getAzimuth();

		// char myArray[3];
		// compass.getDirection(myArray, a);
		
		// Serial.print("A: ");
		// Serial.print(a);
	}


	if (now - AliveTimer > 1000)
	{
		AliveTimer = now;
		led_state_green = !led_state_green;
	}

	if (now - SerialTimer > 250)
	{
		SerialTimer = now;
		// serialPrintStandard();
		// SerialPrintByteArray();
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
	if (now - CheckDistanceTimer > 100)
	{
		CheckDistanceTimer = now;
		CheckDistance();
	}

	// INPUTS
	limitSwitches.left = !digitalRead(limitSwitches.pin_left);
	limitSwitches.right = !digitalRead(limitSwitches.pin_right);

	// OUTPUTS
	digitalWrite(FRONT_LIGHTS_PIN, front_lights_switch);
	digitalWrite(CAMERA_ENABLE_PIN, camera_enable_switch);


	if (analog_left_switch_buffer != analog_left_switch_last)
	{
		PlayToneNGTimer = now;
		play_tone_NG = true;
	}

	if (play_tone_NG)
	{
		if (now - PlayToneNGTimer < 400)
		{
			analogWrite(BUZZER_PIN, 10);
		}
		if (now - PlayToneNGTimer >= 400)
		{
			analogWrite(BUZZER_PIN, 0);
			play_tone_NG = false;
		}
	}

	if (analog_right_switch_buffer != analog_right_switch_last)
	{
		PlayToneOKTimer = now;
		play_tone_OK = true;
	}

	if (play_tone_OK)
	{
		if (now - PlayToneOKTimer < 150)
		{
			analogWrite(BUZZER_PIN, 10);
		}
		if (now - PlayToneOKTimer >= 150 && now - PlayToneOKTimer < 200)
		{
			analogWrite(BUZZER_PIN, 0);
		}
		if (now - PlayToneOKTimer >= 200 && now - PlayToneOKTimer < 350)
		{
			analogWrite(BUZZER_PIN, 10);
		}
		if (now - PlayToneOKTimer >= 350)
		{
			analogWrite(BUZZER_PIN, 0);
			play_tone_OK = false;
		}
	}
		


	digitalWrite(G_LED, led_state_green);

	// COMMUNICATION
	readRadio(0, NOT_PRINTING);

	// MEASURES
	//shunt_measure(CURRENT_MEASURE_PERIOD);

	// MOVE MOTORS
	controlMotors();

	

	//SERIAL DEBUG
	//Serial.println(uint8_t((now - RadioTimeoutTimer) ));

	//WATCHDOGS


	// -------------- END OPERATIONS ---------------------
	//chip_connected_last_state = radio.isChipConnected();
	limitswitches_last = limitSwitches;
	analog_left_switch_last = analog_left_switch_buffer;
	analog_right_switch_last = analog_right_switch_buffer;
}

/*
void SerialDummy()
{
	Serial.print("Uptime (s): ");
	Serial.println(millis() / 1000);
}
*/

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

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

void readRadio(int period, bool printing)
{
	bool camera_enable = false;
	if (radio.available())
	{	
		RadioTimeoutTimer = now;
		radioDataReceive emptyMessage;
		message_receive = emptyMessage;
		last_message_no = current_message_no;
			
		radio.read(&message_receive, sizeof(message_receive));
		current_message_no = message_receive.message_no;
		messages_lost = current_message_no - last_message_no - 1;
		if (printing)
			printMessage(RADIO_PRINT_INCOMING_MESSAGE);
		radio_not_availalble = 0;
		radio_not_availalble_counter = 0;

		analog_left_switch = bitRead(message_receive.bit_array, 0);
		analog_right_switch = bitRead(message_receive.bit_array, 1);
		camera_enable = !bitRead(message_receive.bit_array, 6);	
		front_lights_switch = !bitRead(message_receive.bit_array, 7);	

		if (map(message_receive.potentiometer, 0, 255, 0, 100) > 20)
			velocity_limit = float(map(message_receive.potentiometer, 0, 255, 0, 255));
		else
			velocity_limit = 0.0;

		if (camera_enable)
		{
			CameraOnTimer = millis();
		}

		if (millis() - CameraOnTimer < 5000)
		{
			camera_enable_switch = true;
		}
		else
		{
			camera_enable_switch = false;
		}
	}
	else
	{
		radio_not_availalble = 1;
		radio_not_availalble_counter++;
	}

	if ((message_receive.analog_left_Y == 0 && message_receive.analog_left_X == 0 &&
            message_receive.analog_right_X == 0 && message_receive.analog_right_Y == 0) || 
            (message_receive.analog_left_Y == 255 && message_receive.analog_left_X == 255 &&
            message_receive.analog_right_X == 255 && message_receive.analog_right_Y == 255))
			{
				empty_receive_data = 1;
				front_lights_switch = false;
			}
		
	else
		empty_receive_data = 0;
}