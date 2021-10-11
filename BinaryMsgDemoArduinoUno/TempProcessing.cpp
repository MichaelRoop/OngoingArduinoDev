
#include "TempProcessing.h"
#include <math.h>

TemperatureProcessor::TemperatureProcessor(int sensorValue) {
	this->Init();
	this->ProcessRaw(sensorValue);
}


TemperatureProcessor::TemperatureProcessor() {
	this->Init();
}


void TemperatureProcessor::Init() {
	this->R1 = 10000;
	// steinhart-hart coeficients for thermistor
	this->c1 = 0.001129148;
	this->c2 = 0.000234125;
	this->c3 = 0.0000000876741;
	this->kelvin = 0;
}


TemperatureProcessor::~TemperatureProcessor() {
}


void TemperatureProcessor::ProcessRaw(int sensorValue) {
	// KY-013 analog temperature sensor
	// https://arduinomodules.info/ky-013-analog-temperature-sensor-module/
	float R2 = this->R1 * (1023.0 / (float)sensorValue - 1.0);			 // calculate resistance on thermistor
	float logR2 = log(R2);
	this->kelvin = (1.0 / (this->c1 + this->c2 * logR2 + this->c3 * logR2 * logR2 * logR2)); // temperature in Kelvin
}


float TemperatureProcessor::Kelvin() {
	return this->kelvin;
}


float TemperatureProcessor::Celcius() {
	return this->kelvin - 273.15;
}


float TemperatureProcessor::Farenheit() {
	return (this->Celcius() * 9.0) / 5.0 + 32.0;
}

