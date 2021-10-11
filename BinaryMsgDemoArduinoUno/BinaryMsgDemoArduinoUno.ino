/*
 Name:		BinaryMsgDemoArduinoUno.ino
 Created:	10/11/2021 10:09:24 AM
 Author:	Michael
*/
//#define DEBUG
#ifdef DEBUG
//#define BINARY_MSG_DEBUG
#endif // DEBUG

#include <BinaryMsgProcessor.h>
#include "TempProcessing.h"
#include <SoftwareSerial.h>
#include <BinaryMsgMessages.h>


#ifndef SECTION_DEFINES




// Outgoing msg IDs
#define OUT_MSG_ID_ANALOG_0 20

// MSG IDs for incoming message
#define IN_MSG_ID_LED_RED_PIN 10
#define IN_MSG_ID_LED_BLUE_PIN 11
#define IN_MSG_ID_PMW_PIN_X 12
#define IN_MSG_ID_PMW_PIN_Y 13

// Arduino physical pins
#define LED_RED_PIN 1
#define LED_BLUE_PIN 2
#define PMW_PIN_X 9
#define PMW_PIN_Y 10

// Analog debounce limit
#define ANALOG_DEBOUNCE_GAP 5

// We never copy in more than the max current message size of 12
#define IN_BUFF_SIZE 12

#endif // !SECTION_DEFINES

#ifndef SECTION_VARIABLES

int lastA0Value;
int lastPinX;

// Just have the types you need in your application. One can be used for 
// multiple outgoing IOs by changing the ID and Value before send
MsgFloat32 outFloat;
MsgUInt8 outUint8;
//MsgBool outBool;

// Process the temperature
TemperatureProcessor temperatureProcessor;

// In buffer. Currently largest message is 12 bytes. We can just copy to 
// A specific message in a function to avoid reserving that memory
uint8_t buff[IN_BUFF_SIZE];
uint8_t currentPos = 0;
uint8_t currentRemaining = 0;

// The jumpers on Bluetooth board are set to 4TX and 5RX. 
// They are reversed on serial since RX from BT gets TX to serial
SoftwareSerial btSerial(5, 4); //RX,TX

#endif // !SECTION_VARIABLES

#ifndef SECTION_ARDUINO_FUNCTIONS

void setup() {
#ifdef DEBUG
	Serial.begin(115200);
	while (!Serial) {}
#endif // DEBUG

	// Must set the Bluetooth baud via an AT command AT+UART=115200,1,0
	// otherwise it is not fast enough. SoftwareSerial does corrupts 
	// data at that speed but I can recover. Better to route Bluetooth 
	// to use a hardware serial port but this does for demo
	btSerial.begin(115200); // 

	while (!btSerial) {}
#ifdef DEBUG
	Serial.println("BT Binary...");
#endif // DEBUG

	Initialize();
}

void loop() {
	ListenForData();
	CheckForSendBackData();
}

#endif // !SECTION_ARDUINO_FUNCTIONS

#ifndef SECTION_PRIVATE_HELPERS

// First time initializations
void Initialize() {

	// Digital Pins for demo. Will differ per application
	pinMode(LED_RED_PIN, OUTPUT);
	pinMode(LED_BLUE_PIN, OUTPUT);

	// Digital Pins with PWM for demo. Will differ per application
	pinMode(PMW_PIN_X, OUTPUT);
	pinMode(PMW_PIN_Y, OUTPUT);

	// Analog read values. Out of range forces first value to send
	lastA0Value = 0xFFFFFFFF;
	lastPinX = 0xFFFFFFFF;
	ResetInBuff();

	//-------------------------------------------------------------
	// Register expected id/data type combination for incoming msgs
	// Expecting bool to turn LED on or off
	BinaryMsgProcessor::RegisterInIds(IN_MSG_ID_LED_BLUE_PIN, typeBool);
	BinaryMsgProcessor::RegisterInIds(IN_MSG_ID_LED_RED_PIN, typeBool);

	// Expectin Uint 8 msg with values 0-255 for PWM on digital pin
	BinaryMsgProcessor::RegisterInIds(IN_MSG_ID_PMW_PIN_X, typeUInt8);
	BinaryMsgProcessor::RegisterInIds(IN_MSG_ID_PMW_PIN_Y, typeUInt8);

	// Will be raised from parsed incoming message if msg value is of certain type
	// Register all you require for your program
	BinaryMsgProcessor::RegisterFuncBool(&CallbackBoolValue);
	BinaryMsgProcessor::RegisterFuncUInt8(&CallbackUint8Value);
#ifdef DEBUG
	BinaryMsgProcessor::RegisterErrCallback(&ErrCallback);
#endif // DEBUG
}


void ResetInBuff() {
	memset(buff, 0, IN_BUFF_SIZE);
	currentPos = 0;
	currentRemaining = 0;
}


// Purge all in the app buffer and Bluetooth serial buffer
void PurgeBuffAndBT() {
	int available = btSerial.available();
	while (available > 0) {
		// Iterate through max BUFF size read at a time
		if (available > IN_BUFF_SIZE) {
			available = IN_BUFF_SIZE;
		}
		int count = btSerial.readBytes(buff, available);
		available = btSerial.available();
	}
	ResetInBuff();
}

#endif // !SECTION_PRIVATE_HELPERS

#ifndef SECTION_INCOMING_MSGS

void ListenForData() {
	// Loop through until nothing in buffer to minimize overflow
	int available = btSerial.available();
	while (available > 0) {
		available = btSerial.available();
		if (available > 0) {
			if (currentPos == 0) {
				GetNewMsg(available);
			}
			else {
				GetRemainingMsgFragment(available);
			}
		}
	}
}


// New message arriving. Don't pick up until the entire header is in BT buffer
void GetNewMsg(int available) {
	if (available >= MSG_HEADER_SIZE) {
		//Serial.print("GetNewMsg:"); Serial.println(available);
		currentPos = btSerial.readBytes(buff, MSG_HEADER_SIZE);
		if (BinaryMsgProcessor::ValidateHeader(buff, currentPos)) {
			currentRemaining = (BinaryMsgProcessor::GetSizeFromHeader(buff) - MSG_HEADER_SIZE);
			available = btSerial.available();
			if (available >= currentRemaining) {
				GetRemainingMsgFragment(available);
			}
		}
		else {
#ifdef DEBUG
			Serial.print("GetNewMsgERR- currentPos:"); Serial.println(currentPos);
			Serial.print("---:");
			Serial.print(buff[0]); Serial.print(",");
			Serial.print(buff[1]); Serial.print(",");
			Serial.print(buff[2]); Serial.print(",");
			Serial.print(buff[3]); Serial.print(",");
			Serial.print(buff[4]); Serial.print(",");
			Serial.print(buff[5]); Serial.print(",");
			Serial.print(buff[6]); Serial.print(",");
			Serial.println(buff[8]);
#endif // DEBUG
			PurgeBuffAndBT();
		}
	}
}


// Get enough bytes to make a completed message and process result
void GetRemainingMsgFragment(int available) {
	if (available >= currentRemaining) {
		size_t count = btSerial.readBytes(buff + currentPos, currentRemaining);
		//Serial.print("GetFrag:"); Serial.print(currentRemaining); Serial.print(":"); Serial.println(count);
		currentPos += count;
		bool result = BinaryMsgProcessor::ValidateMessage(buff, currentPos);
		ResetInBuff();
	}
}

#endif // !SECTION_INCOMING_MSGS

#ifndef SECTION_OUTGOING_MSGS

// May have to put these in a stack where the send can happend when other sends complete
void CheckForSendBackData() {
	// In our demo, a KY-013 temperature sensor is attached to Analog pin A0
	if (ChatterFiltered(analogRead(A0), &lastA0Value, OUT_MSG_ID_ANALOG_0)) {
		SendTemperature(lastA0Value);
	}


	//if (ChatterFiltered(analogRead(PMW_PIN_X), &lastPinX, IN_MSG_ID_PMW_PIN_X)) {
	//	SendUint8Msg(IN_MSG_ID_PMW_PIN_X, lastPinX);
	//}
	//int val = analogRead(PMW_PIN_X);
	//if (val != lastPinX) {
	//	lastPinX = val;
	//	SendUint8Msg(IN_MSG_ID_PMW_PIN_X, val);
	//}

	// TODO - put the PWM 12 here
}


bool ChatterFiltered(int current, int* last, uint8_t pinId) {
	if ((current - ANALOG_DEBOUNCE_GAP) > *last ||
		(current + ANALOG_DEBOUNCE_GAP) < *last) {
		*last = current;
		return true;
	}
	return false;
}

/*
// Use any of the following with the specified messge to send info to Dashboard
void SendBoolMsg(uint8_t id, bool value) {
	outBool.Id = id;
	outBool.Value = value;
	SendMsg(&outBool, outBool.Size);
}

void SendInt8Msg(uint8_t id, int8_t value) {
}

void SendInt16Msg(uint8_t id, int16_t value) {
}

void SendInt32Msg(uint8_t id, int32_t value) {
}

void SendUInt8Msg(uint8_t id, uint8_t value) {
}

void SendUInt16Msg(uint8_t id, uint16_t value) {
}

void SendUInt32Msg(uint8_t id, uint32_t value) {
}
*/

void SendFloatMsg(uint8_t id, float value) {
	outFloat.Id = id;
	outFloat.Value = value;
	//Serial.println(value);
	SendMsg(&outFloat, outFloat.Size);
}


void SendUint8Msg(uint8_t id, uint8_t value) {
	outUint8.Id = id;
	outUint8.Value = value;
	//Serial.println(value);
	SendMsg(&outUint8, outUint8.Size);
}


void SendMsg(void* msg, int size) {
	size_t sent = btSerial.write((uint8_t*)msg, size);
	//Serial.print("Size sent:"); Serial.println(sent);
}


// Convert raw sensor to temperature
void SendTemperature(int sensorValue) {
	temperatureProcessor.ProcessRaw(sensorValue);
	SendFloatMsg(OUT_MSG_ID_ANALOG_0, temperatureProcessor.Celcius());
	//SendFloatMsg(ANALOG_0_ID, tempProcessor.Kelvin());
	//SendFloatMsg(ANALOG_0_ID, tempProcessor.Farenheit());
}

#endif // !SECTION_OUTGOING_MSGS

#ifndef SECTION_CALLBACKS

#ifdef DEBUG
void ErrCallback(ErrMsg* errMsg) {
	PrintErr(errMsg);
	if (errMsg->Error != err_NoErr) {
		Serial.print("-SOH:"); Serial.println(errMsg->SOH);
		Serial.print("-STX:"); Serial.println(errMsg->STX);
		Serial.print("-Size:"); Serial.println(errMsg->Size);
		Serial.print("-Type:"); PrintDataType(errMsg);
		Serial.print("-Payload Size:"); Serial.println(errMsg->PayloadSize);
		Serial.print("-Required Size:"); Serial.println(errMsg->RequiredSize);
		Serial.print("-ID:"); Serial.println(errMsg->Id);
	}
}


void PrintDataType(ErrMsg* msg) {
	switch (msg->DataType) {
	case typeUndefined:
	case typeBool:
		Serial.println("bool");
		break;
	case typeInt8:
		Serial.println("int8");
		break;
	case typeUInt8:
		Serial.println("uint8");
		break;
	case typeInt16:
		Serial.println("int16");
		break;
	case typeUInt16:
		Serial.println("uint16");
		break;
	case typeInt32:
		Serial.println("int32");
		break;
	case typeUInt32:
		Serial.println("uint32");
		break;
	case typeFloat32:
		Serial.println("float32");
		break;
		//case typeString:
	case typeInvalid:
		Serial.println("invalid type");
		break;
	default:
		Serial.println("unhhandled type");
		break;
		break;
	}
}


void PrintErr(ErrMsg* msg) {
	switch (msg->Error) {
	case err_NoErr:
		Serial.println("\nNo err");
		break;
	case err_InvalidType:
		Serial.println("\nInvalid data type");
		break;
	case err_InvalidHeaderSize:
		Serial.println("\nInvalid header size");
		break;
	case err_StartDelimiters:
		Serial.println("\nErr with start delimiters");
		break;
	case err_InvalidSizeField:
		Serial.println("\nInvalid msg size value");
		break;
	case err_InvalidPayloadSizeField:
		Serial.println("\nBad payload size value");
		break;
	case err_InvalidDataTypeForRegisteredId:
		Serial.println("\nInvalid data type for msg");
		break;
	case err_CallbackNotRegisteredForId:
		Serial.println("\nNo callback for msg");
		break;
	default:
		Serial.println("\nUnhandled");
		break;
	}
}
#endif


void CallbackBoolValue(uint8_t id, bool value) {
	//Serial.print("bool-id:"); Serial.print(id); Serial.print(" Val:"); Serial.println(value);
	switch (id) {
	case IN_MSG_ID_LED_RED_PIN:
		digitalWrite(LED_RED_PIN, value ? HIGH : LOW);
		break;
	case IN_MSG_ID_LED_BLUE_PIN:
		digitalWrite(LED_BLUE_PIN, value ? HIGH : LOW);
		break;
	default:
		// TODO - error msg if desired
		break;
	}
}


void CallbackUint8Value(uint8_t id, uint8_t value) {
	// Debug only 
	//Serial.print("U8-id:"); Serial.print(id); Serial.print(" Val:"); Serial.println(value);
	// Analog writes from 0 - 255 (8 bits), so we use UInt8
	// Reads from 0 - 1023 (10 bits)
	switch (id) {
	case IN_MSG_ID_PMW_PIN_X:
		// Write to pin
		analogWrite(PMW_PIN_X, value);
		// For demo, bounce back the new value
		SendUint8Msg(IN_MSG_ID_PMW_PIN_X, value);
		break;
	case IN_MSG_ID_PMW_PIN_Y:
		analogWrite(PMW_PIN_Y, value);
		break;
	default:
		// TODO - error msg if desired
		break;
	}
}

#endif // !SECTION_CALLBACKS
