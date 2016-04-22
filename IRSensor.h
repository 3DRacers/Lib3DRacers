#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Config.h"
#include <Arduino.h>

class IRSensor
{

public:

	int threshold; //exclusive
	int preSilenceRequired; //1 s
	int postSilenceRequired; //1 s
	int maxSequenceDuration; //1 s
	short maxPeaksAllowed;
	float peaksDistanceVariationAllowed; //ms

	unsigned long lastSequenceStart;
	unsigned long lastSequenceDuration;
	
	unsigned long lastSilence;
	unsigned long sequenceStart;
	unsigned long peakTime;
	unsigned long peakTimes[5];
	int peaks;
	int lastPeak;

	IRSensor();
	bool process(short sensor);

private:
	bool _process(short sensor);
	int peakDetection(int sensorValue);
	void sendPlotData(String seriesName, int data);
};

#endif