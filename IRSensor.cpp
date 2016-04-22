#ifndef IRSENSOR_C
#define IRSENSOR_C

#include "IRSensor.h"
#include <Arduino.h>

//Uncomment to enable Sensor debugging with MegunoLink plotter software
//#define DEBUG_SENSOR true

#ifdef DEBUG_SENSOR
	#define DBG_SENSOR sendPlotData
#else
	#define DBG_SENSOR 
#endif

IRSensor::IRSensor():
	threshold(990),
	preSilenceRequired(1000), 
	postSilenceRequired(1000),
	maxSequenceDuration(1000), //1 s
	maxPeaksAllowed(3),
	peaksDistanceVariationAllowed(250), //ms

	lastSequenceStart(0),
	lastSequenceDuration(0),
	
	lastSilence(0),
	sequenceStart(0),
	peakTime(0),

	peaks(0),
	lastPeak(0)
{ 

}

bool IRSensor::process(short sensor) {
	bool val = _process(sensor);
	if (val) {
		DBG_SENSOR("sequence", 1025);
    }
	else {
		DBG_SENSOR("sequence", threshold);
	}
	DBG_SENSOR("raw", sensor);
	
	return val;
}

bool IRSensor::_process(short sensor)
{ 
  unsigned long myTime = millis();
  
  //Sequence is running for too much:
  if(sequenceStart != 0 && peaks != maxPeaksAllowed && myTime - sequenceStart > maxSequenceDuration) {
    //Forcibly ends the sequence
    peaks = 0;
    sequenceStart = 0;
	DBG_SENSOR("ERR_TOO_LONG", 1025);
  }
  else if(sequenceStart != 0 && peaks == maxPeaksAllowed && lastSilence != 0 && myTime - lastSilence > postSilenceRequired) {
    //Possibly Valid Sequence Found: all peaks received, silence detected and for more than postSilenceRequired
    
	DBG_SENSOR("peaks", threshold); //Null peaks graph
	
    //Check peaks validity:
    int avgD = 0;
    int maxD = 0;
    int d = 0;
    for(int c=1; c < peaks; c++) { //NB: forse qua dovrei escludere 1 picco con il maxD dalla media, per non alterarla
      int d = peakTimes[c] - peakTimes[c-1];
      if(d > maxD) {
        maxD = d;
      }
      avgD += d;
    }
    avgD = 1.0 * avgD / peaks;

    if(maxD > avgD + peaksDistanceVariationAllowed) {
	  //Ends the current sequence:
	  DBG_SENSOR("rejected", 1025);
	  peaks = 0;
	  sequenceStart = 0;
      return false; //NOT VALID, there is an irregular peak 
    }
    else {      
	  lastSequenceDuration = peakTimes[peaks-1] - sequenceStart;
      lastSequenceStart = sequenceStart;
	
	  //Ends the current sequence:
	  peaks = 0;
	  sequenceStart = 0;
      return true; //VALID: al peaks in range d
    }
      
  }
  else {
    //Look for peaks:
    int peakValue = peakDetection(sensor);
    if(peakValue != -1) {

      //Peak found
	  DBG_SENSOR("peaks", peakValue);
      if(sequenceStart == 0) {
        if(myTime - lastSilence > preSilenceRequired) {
          
          //We have a new sequence!
          peakTimes[peaks] = peakTime; //NB: we use peak time becuse peakDetection return when the edge goes low (at the end of th peak curve)
          peaks++;    
          sequenceStart = peakTime;    
        }
		else {
			DBG_SENSOR("ERR_NO_SILENCE", 1025);
		}

      }
      else if(peaks > maxPeaksAllowed) {
          //Forcibly ends the sequence
          peaks = 0;
          sequenceStart = 0;
      }
      else {
        //Continue the sequence
        peakTimes[peaks] = peakTime;
        peaks++;       
      }
    }
	else {
	  DBG_SENSOR("peaks", threshold);      
	}
    
  }


  //Edge detect of sensor:
  if(sensor > threshold) {
    lastSilence = 0;  
	DBG_SENSOR("lastSilence", threshold);  
  }
  else {
    if(lastSilence == 0) {
      lastSilence = myTime; 
	  DBG_SENSOR("lastSilence", 1025);
    }
  }
  
  return false;
}

//For Meguno link
void IRSensor::sendPlotData(String seriesName, int data)
{
	#ifdef DEBUG_SENSOR
	  Serial.print("{");
	  Serial.print(seriesName);
	  Serial.print(",T,");
	  Serial.print((float) data);
	  Serial.println("}");
	#endif
}

int IRSensor::peakDetection(int sensorValue) {
  if (sensorValue > lastPeak) {
    lastPeak = sensorValue;
    peakTime = millis();
  }
  if (sensorValue <= threshold ) {
    if (lastPeak > threshold) {
      // you have a peak value:
      int peakValue = lastPeak;
      // reset the peak value:
      lastPeak = 0;
      return peakValue;
    }
  }
  return -1;
}


#endif