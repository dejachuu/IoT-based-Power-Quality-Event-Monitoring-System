#include <Filters.h>
#include<SoftwareSerial.h>
#include "ACS712.h"
#include "arduinoFFT.h"

arduinoFFT FFT;
#define CHANNEL A0
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 100; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

SoftwareSerial Arduino_SoftSerial (10,11);   // RX,TX

ACS712 sensor(ACS712_05B, A1);

float I;

float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist

int Sensor = 0; //Sensor analog input, here it's A0

float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage

unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;

void setup() {
  //Open serial communication (Arduino - PC)

  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  Serial.begin(57600);

  sensor.calibrate();

  //Open Serial communication (Arduino - NodeMCU) 
  
  Arduino_SoftSerial.begin(9600);
}

void loop() {
RunningStatistics inputStats;                //Easy life lines, actual calculation of the RMS requires a load of coding
inputStats.setWindowSecs( windowLength );
   
while(true){

microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  /* Print the results of the sampling according to time */
  //Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  //Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  //Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak();
  //Serial.println(x, 6); //Print out what frequency is the most dominant.
    
Sensor = analogRead(A0);  // read the analog in value:
inputStats.input(Sensor);  // log to Stats function
        
if((unsigned long)(millis() - previousMillis) >= printPeriod) {
previousMillis = millis();   // update time every second
            
Serial.print( "\n" );
      
current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
current_Volts= current_Volts*(40.3231);

I = sensor.getCurrentAC();
      
Arduino_SoftSerial.print(current_Volts,2);      Arduino_SoftSerial.print("A");
Arduino_SoftSerial.print(I,2);      Arduino_SoftSerial.print("B");
Arduino_SoftSerial.print(x,2);      Arduino_SoftSerial.print("C"); 
Arduino_SoftSerial.print("\n");
}
}
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
  }
}
                 
