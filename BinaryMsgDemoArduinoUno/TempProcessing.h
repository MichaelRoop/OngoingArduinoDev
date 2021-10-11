// TempProcessing.h

#ifndef _TEMPPROCESSING_h
#define _TEMPPROCESSING_h

class TemperatureProcessor {
public:
	TemperatureProcessor();
	TemperatureProcessor(int sensorValue);
	~TemperatureProcessor();

	void ProcessRaw(int sensorValue);
	float Kelvin();
	float Celcius();
	float Farenheit();
private:
	// value of R1 on board
	float R1;
	// steinhart-hart coeficients for thermistor
	float c1;
	float c2;
	float c3;
	float kelvin;

	void Init();
};


#endif

