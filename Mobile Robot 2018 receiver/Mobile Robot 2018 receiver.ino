#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//SWITCHES
#define RADIO_PRINT_INCOMING_MESSAGE 1

//TIMERS
#define ONE_MS 1
#define CURRENT_MEASURE_PERIOD 50
#define SERIAL_PRINT_PERIOD 1000
#define ANALOG_CALIBRATION_TIME 1500

//CIRCULAR BUFFERS
#define CURRENT_BUFFER_SIZE 20
#define ANALOG_CALIBRATION 1

//PHYSICAL PINS
#define SHUNT_PIN_PLUS A1
#define SHUNT_PIN_MINUS A2

//RADIO CONFIG
#define CE 7
#define CSN 8

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
	byte reserved0;
	byte reserved1;
	byte reserved2;
	byte reserved3;
	byte message_no;
};




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
byte speed_left;
byte speed_right;




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

//current measure
float current_array[CURRENT_BUFFER_SIZE];
float current_shunt;
float average_current;

void setup()
{
	Serial.begin(9600);
	Serial.println("Starting...");
	radioConfig();
	pinMode(R_LED, OUTPUT);
	Serial.println("Setup completed");
}

void loop()
{
	if (radio.isChipConnected() == 1 && chip_connected_last_state == 0)
		radioConfig();
/*
	if (messages_lost > 4)
	{
		Serial.println("Radio restart");
		radio.powerDown();
		delay(50);
		radioConfig();
		radio.powerUp();		
	}
*/
	
	now = millis();
	readRadio(0);
	shunt_measure(CURRENT_MEASURE_PERIOD);
	serialPrint(SERIAL_PRINT_PERIOD);

	if (messages_lost > 2)
	{
		LeftMotor.stop();
		RightMotor.stop();
	}

	if (message.message_no != 0 && !empty_receive_data)
	{
		switch (message.analog_left_Y)
		{
		case 0 ... 90:
			LeftMotor.backward(80);
			break;
		case 91 ... 180:
			LeftMotor.stop();
			break;
		case 181 ... 255:
			LeftMotor.forward(80);
			break;
		}

		switch (message.analog_right_Y)
		{
		case 0 ... 90:
			RightMotor.backward(80);
			break;
		case 91 ... 180:
			RightMotor.stop();
			break;
		case 181 ... 255:
			RightMotor.forward(80);
			break;
		}
	}

	if (message.analog_left_Y == 0 && message.analog_left_X && message.analog_right_X == 0 & message.analog_right_Y == 0)
		empty_receive_data = 1;
	else
		empty_receive_data = 0;

	if (empty_receive_data == 1)
		digitalWrite(R_LED, 1);
	else
		digitalWrite(R_LED, 0);



	//analogCalibration(now, ANALOG_CALIBRATION_TIME, analogLeftYrest, analogLeftXrest, analogRightYrest, analogRightXrest, done_calibration);
	/*
	if (empty_receive_data = 0)
	{
		if (message.analog_left_Y > 160 && current_message_no != 0)
		{
			LeftMotor.forward(100);

		}
		else if (message.analog_left_Y < 100 && current_message_no != 0)
		{
			LeftMotor.backward(100);
		}
		else
			LeftMotor.stop();

		if (message.analog_right_Y > 160 && current_message_no != 0)
		{
			RightMotor.forward(100);
		}
		else if (message.analog_right_Y < 100 && current_message_no != 0)
		{
			RightMotor.backward(100);
		}
		else
			RightMotor.stop();
	}
	*/

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

