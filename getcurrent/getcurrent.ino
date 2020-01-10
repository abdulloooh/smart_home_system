const unsigned int numReadings = 500; //samples to calculate Vrms.

int readingsVClamp[numReadings];    // samples of the sensor SCT-013-000
int readingsGND[numReadings];      // samples of the virtual ground
float SumSqGND = 0;            
float SumSqVClamp = 0;
float total = 0; 


int PinVClamp = 34;    // Sensor SCT-013-000
int PinVirtGND = 32;   // Virtual ground

void setup() {
  Serial.begin(115200);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsVClamp[thisReading] = 0;
    readingsGND[thisReading] = 0;
  }
}

void loop() {
  unsigned int i=0;
  SumSqGND = 0;
  SumSqVClamp = 0;
  total = 0; 
  
  for (unsigned int i=0; i<numReadings; i++)
  {
    readingsVClamp[i] = analogRead(PinVClamp) - analogRead(PinVirtGND);
    delay(1); // 
  }

  //Calculate Vrms
  for (unsigned int i=0; i<numReadings; i++)
  {
    SumSqVClamp = SumSqVClamp + sq((float)readingsVClamp[i]);

  }
  
  total = sqrt(SumSqVClamp/numReadings);
  total= (total*(float)250/32768); // Rburden=3300 ohms, LBS= 0,004882 V (5/1024)
                             // Transformer of 2000 laps (SCT-013-000).
      //using 12-bit ADC => 4096 and 3200ohms                      // 5*220*2000/(3300*1024)= 2/3 (aprox)
  
  Serial.println(total);
  delay(1500);  
}
