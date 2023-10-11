#ifndef SHM_h
#define SHM_h

/********** LIBRARIES **********/
#include "Arduino.h"
#include <algorithm>
#include <math.h>
#include <arduinoFFT.h>
#include <LoRa.h>
#include <stdio.h>
#include <iostream> 
#include <RTCZero.h>
#include "DHT.h"
#include "ADXL335.h"

/********** MEASUREMENT CONSTANTS **********/
static constexpr double G = 9.80665; // Gravitational Constant
static constexpr double pi = 3.14159265358979323846; 

/********** ACCELEROMETER THRESHOLDS **********/
const double xNoise = 0.01;
const double yNoise = 0.01;
const double zNoise = 0.01;

/********** ACCELEROMETER REFERENCE VALUES **********/
// const double vRef = 3.3; //3.3V reference / supply
const double vRef = 3.0; // BATTERY CONNECTION

/********** SAMPLING PARAMETERS **********/
static const int f = 2; // Frequency Target
static const int maxFreq = 25; //Max Frequency Detected by FFT
static const uint16_t N = 512; //MUST BE EXP 2
static constexpr double Fs = maxFreq * 2; // Nyquist Sampling Freq = 2 * MaxFrequency
static const double Ts = 1.0 / Fs; //Sampling Period
static const double sampleTime = (Ts * 1000); //Sampling Period in Miliseconds

/********** FILTER PARAMETERS **********/
constexpr double Fc = 5; // Cutoff Frequency

//Period for sampling
const long period = 10000; //10 Sec

//Accelerometer Constructor for ADXL335 Sensor
ADXL335 accelerometer;

// Create an rtc object
RTCZero rt_clock;

void read_acceleration(double xAccel[], double yAccel[], double zAccel[]) {
  float ax, ay, az; 
  //Loop through samples
  for (int i = 0; i < N; i++) {
    //Takes Readings from ADXL335 Sensor
    accelerometer.getAcceleration(&ax, &ay, &az);

    // // Apply threshold to remove noise ***DETERMINE THRESHOLGS***
    ax = (abs(ax) <= xNoise) ? 0 : ax;
    ay = (abs(ay) <= yNoise) ? 0 : ay;
    az = (abs(az) <= zNoise) ? 0 : az;

    //Converts G's to m/s/s
    xAccel[i] = ax * G;
    yAccel[i] = ay * G;
    zAccel[i] = az * G;

    //Delay between samples
    delay(Ts*1000);
  }
}


double findMax(const double data[]) {
  double maxVal = 0;
  for (int i = 0; i < N; ++i) {
    double absVal = abs(data[i]);
    if (absVal > abs(maxVal)) {
      maxVal = data[i];
    }
  }
  return maxVal;
}

void checkNan(float& freq) {
 if (isnan(freq)) {
  freq = 0;
 }
}

void LPF(double *data) {
	float alpha = 1.0 / ((2.0*PI*Fc*Ts) + 1.0);
	double output = data[0];
	for (uint16_t i = 1; i < N; i++)
	{
		output = alpha*data[i] + (1-alpha) * output;
    data[i] = output;
	}
}


#endif