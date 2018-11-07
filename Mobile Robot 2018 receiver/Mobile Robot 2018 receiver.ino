#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//SWITCHES
#define RADIO_PRINT_INCOMING_MESSAGE 0

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

//RADIO MESSAGE PARTS
#define	ANALOG_LEFT_Y 0
#define ANALOG_LEFT_X 1
#define ANALOG_RIGHT_Y 2
#define ANALOG_RIGHT_X 3
#define	CONTROL_BYTE 4
#define MESSAGE_NO 5


//radio variables
RF24 radio(14, 15);
const byte rxAddr[6] = {'1','N','o','d','e','0'};
byte incoming_message[6];

short last_message_no, current_message_no, messages_lost;

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

	radio.begin();
	radio.setChannel(2);
	radio.openReadingPipe(0, rxAddr);
	radio.startListening();
}

void loop()
{
	now = millis();
	readRadio(0);
	shunt_measure(CURRENT_MEASURE_PERIOD);
	serialPrint(SERIAL_PRINT_PERIOD);

	analogCalibration(now, ANALOG_CALIBRATION_TIME, analogLeftYrest, analogLeftXrest, analogRightYrest, analogRightXrest, done_calibration);
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
