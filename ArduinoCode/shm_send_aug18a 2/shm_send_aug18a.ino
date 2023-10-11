#include "arduino_secrets.h"
//////////////////////////////////////
/**********   SENDER CODE   *********/
//////////////////////////////////////
#include "thingProperties.h"
#include "SHM.h"

void setup() {
   // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  //Info About Connection (0 - 4) - Higher Number means more Information
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  //Start Accelerometer Readings
  accelerometer.begin();

  //Master Sync Request
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // send sync packet
  LoRa.beginPacket();
  int start = millis();
  LoRa.print(rt_clock.getEpoch());
  LoRa.endPacket();
  //Note: Only use delay if transmission length is in the hundreds of meters
  //delay(millis() - start);
  
  Serial.println("Synched");
}

void loop() {
  ArduinoCloud.update();

  // ACCELERATION SAMPLING
  double xAccel[N];
  double yAccel[N];
  double zAccel[N];

  // MAXIMUM ACCELERATION
  float zAccelMax = 0.0;

  // MAXIMUM FREQUENCY
  float zFreq = 0.0;
  
  // PHASE SHIFT
  float zPhase = 0.0;
  
  //Array's for imaginary components of FFT to prevent overflow.
  double imagZ[N] = {};

  //Read sensor values and determine acceleration
  read_acceleration(xAccel, yAccel, zAccel);

  //Apply Low Pass Filter to data to remove High Freq Components
  LPF(zAccel);

  //Determine Max Acceleration from sensor values.
  zAccelMax = float(findMax(zAccel));

  //FFT Constructor For Each Direction of Accelerometer
  arduinoFFT Z_FFT(zAccel, imagZ, N, Fs);

  //Remove DC Component Of the data.
  Z_FFT.DCRemoval();

  //Apply Hamming window to the data to improve resolution
  Z_FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  //Compute the FFT of the data
  Z_FFT.Compute(FFT_FORWARD);
  
  //Find Maximum Frequency Of Data
  zFreq = float(Z_FFT.MajorPeak());
    
  Serial.print("Peak Freq: ");
  Serial.println(float(Z_FFT.MajorPeak()));
  
  //Find Bin of Peak Frequency 
  int binZ = zFreq * N / Fs;
  
  //Compute Phase Shift of Max Freq
  zPhase = atan2(imagZ[binZ],zAccel[binZ]);
  
  Serial.print("Peak Phase: ");
  Serial.println(atan2(imagZ[binZ],zAccel[binZ]));

  //Check for NaN values in the frequency outputs.
  checkNan(zFreq);
  Serial.println("Phase: ");
  for(int i=0; i<N; i++){
    Serial.println(atan2(imagZ[i],zAccel[i]));
  }
  Serial.println("AMPLITUDE: ");
    for(int i=0; i<N; i++){
    Serial.println(sqrt(zAccel[i] * zAccel[i] + imagZ[i] * imagZ[i]));
  }
  
  // Update Arduino IoT Cloud Variables
  freqz = zFreq;
  phasez = zPhase;
  
  //Delay between samples
  float start = millis();
  while((millis() - start) < period);
}