#include <Arduino.h>
#include <PubSubClient.h>
//#include "EmonLib.h"
#include "WiFi.h"
#include <driver/adc.h>
// The GPIO pin were the CT sensor is connected to (should be an ADC input)
//#define ADC_INPUT 34
#define HOME_VOLTAG 220.0
// Force EmonLib to use 10bit ADC resolution
#define ADC_BITS    10
//define cummulative energy and its time
unsigned long timeCount = 0;
double cummulativeEnergy = 0;
double sessionEnergy;
double energyLeft;
//double targetConsumption;
//double UNITCOST = 25;
double cummCost;
double sessionCost;
double percentUsage;
//double target;
double refEnergy = 0; //reference energy when target is set
// Create instances
//EnergyMonitor emon1;
//*******************************
//MQTT
#define TOKEN "BBFF-4OMP5F0TB4SvzDCDMgRQVrePZBy6gR" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "abdulloooh" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
//it should be a random and unique ascii string and different from all other devices
/****************************************
   Define Constants
 ****************************************/
#define VARIABLE_LABEL_1 "power" // Assing the variable label
#define VARIABLE_LABEL_2 "energy" // Assing the variable label
#define VARIABLE_LABEL_3 "cummcost"
#define VARIABLE_LABEL_4 "energyleft"
#define VARIABLE_LABEL_5 "sessioncost"
#define VARIABLE_LABEL_6 "percentusage"
#define VARIABLE_LABEL_7 "sessionenergy"
#define VARIABLE_LABEL_SUB_1 "relay1" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_2 "relay2" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_3 "relay3" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_4 "relay4" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_5 "target" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_6 "unitcost" //total cost
#define VARIABLE_LABEL_SUB_7 "reset"
#define DEVICE_LABEL "esp32" // Assig the device label

#define R1 26 // Set the GPIO26 as RELAY
#define R2 27 // Set the GPIO27 as RELAY2
#define R3 14
#define R4 12

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[700];
char topic[1000];
//char topicSubscribe[100];
// Space to store values to send
char str_power[10];
char str_energy[10];
char str_cummcost[10];
char str_energyleft[10];
char str_sessionenergy[10];
char str_sessioncost[10];
char str_percentusage[10];

const int ERROR_VALUE = 65535;  // Set here an error value

const uint8_t NUMBER_OF_VARIABLES = 7; // Number of variable to subscribe to
char * variable_labels[NUMBER_OF_VARIABLES] = {"relay1", "relay2", "relay3", "relay4" , "target", "unitcost", "reset"}; // labels of the variable to subscribe to

float CONTROL1; // Name of the variable to be used within the code.
float CONTROL2; // Name of the variable to be used within the code.
float CONTROL3; // Name of the variable to be used within the code.
float CONTROL4; // Name of the variable to be used within the code.
float TARGET = 10000; // Name of the variable to be used within the code.//Default Target is 100units = 100kwh
float UNITCOST = 25;
float RESET = 1;
float value; // To store incoming value.
uint8_t variable; // To keep track of the state machine and be able to use the switch case.

/*current reading*/
const unsigned int numReadings = 500; //samples to calculate Vrms.

int readingsVClamp[numReadings];    // samples of the sensor SCT-013-000
int readingsGND[numReadings];      // samples of the virtual ground
float SumSqGND = 0;
float SumSqVClamp = 0;
float total = 0;


int PinVClamp = 34;    // Sensor SCT-013-000
int PinVirtGND = 32;   // Virtual ground

/****************************************
   Initializate constructors for objects
 ****************************************/

WiFiClient ubidots;
PubSubClient client(ubidots);
/****************************************
   Auxiliar Functions
 ****************************************/

void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "") ) {
      Serial.println("Connected");
      //      client.subscribe(topicSubscribe);
    }
    else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      //loop through the code and come back again;
      //Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      //delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  char* variable_label = (char *) malloc(sizeof(char) * 30);;
  get_variable_label_topic(topic, variable_label);
  value = btof(payload, length);
  set_state(variable_label);
  execute_cases();
  free(variable_label);
}

// Parse topic to extract the variable label which changed value
void get_variable_label_topic(char * topic, char * variable_label) {
  //  Serial.print("topic:");
  //  Serial.println(topic);
  sprintf(variable_label, "");
  for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
    char * result_lv = strstr(topic, variable_labels[i]);
    if (result_lv != NULL) {
      uint8_t len = strlen(result_lv);
      char result[100];
      uint8_t i = 0;
      for (i = 0; i < len - 3; i++) {
        result[i] = result_lv[i];
      }
      result[i] = '\0';
      //      Serial.print("Label is: ");
      //      Serial.println(result);
      sprintf(variable_label, "%s", result);
      break;
    }
  }
}



// cast from an array of chars to float value.
float btof(byte * payload, unsigned int length) {
  char * demo_ = (char *) malloc(sizeof(char) * 10);
  for (int i = 0; i < length; i++) {
    demo_[i] = payload[i];
  }
  return atof(demo_);
}

// State machine to use switch case
void set_state(char* variable_label) {
  variable = 0;
  for (uint8_t i = 0; i < NUMBER_OF_VARIABLES; i++) {
    if (strcmp(variable_label, variable_labels[i]) == 0) {
      break;
    }
    variable++;
  }
  if (variable >= NUMBER_OF_VARIABLES) variable = ERROR_VALUE; // Not valid
}


// Function with switch case to determine which variable changed and assigned the value accordingly to the code variable
void execute_cases() {
  switch (variable) {
    case 0:
      Serial.println("Switch 1 reporting...");
      CONTROL1 = value;
      digitalWrite(R1, value);
      Serial.print("CONTROL1: ");
      Serial.println(CONTROL1);
      Serial.println();
      break;
    case 1:
      Serial.println("Switch 2 reporting...");
      CONTROL2 = value;
      digitalWrite(R2, value);
      Serial.print("CONTROL2: ");
      Serial.println(CONTROL2);
      Serial.println();
      break;
    case 2:
      Serial.println("Switch 3 reporting...");
      CONTROL3 = value;
      digitalWrite(R3, value);
      Serial.print("CONTROL3: ");
      Serial.println(CONTROL3);
      Serial.println();
      break;
    case 3:
      Serial.println("Switch 4 reporting...");
      CONTROL4 = value;
      digitalWrite(R4, value);
      Serial.print("CONTROL4: ");
      Serial.println(CONTROL4);
      Serial.println();
      break;
    case 4:
      if (value != TARGET) {
        Serial.println("Target consumption reporting...");
        TARGET = value; //set target consumption
        refEnergy = cummulativeEnergy; // upfate reference energy
        Serial.print("Reference energy set at: ");
        Serial.println(cummulativeEnergy);
        Serial.print("TARGET CONSUMPTION: ");
        Serial.println(TARGET);
        Serial.println();
      }
      break;
    case 5:
      if (value != UNITCOST) {
        Serial.println("Unit cost set value reporting...");
        UNITCOST = value; //Set global variable unit cost
        Serial.print("UNIT COST: ");
        Serial.println(UNITCOST);
        Serial.println();
      }
      break;
    case 6:
      if (value != RESET) {
        Serial.println("Reset reporting...");
        RESET = value;
        Serial.print("RESET: ");
        Serial.println(RESET);
        if (RESET == 1) {
          cummulativeEnergy = refEnergy = sessionEnergy = cummCost = sessionCost = percentUsage = energyLeft = 0;
          Serial.println("System reset");
          Serial.println();
        }
      }
      break;
    //    case 7:
    //      CONTROL8 = value;
    //        digitalWrite(R8,value);
    //      Serial.print("CONTROL8: ");
    //      Serial.println(CONTROL8);
    //      Serial.println();
    //      break;
    //    case 8:
    //      CONTROL9 = value;
    //        digitalWrite(R9,value);
    //      Serial.print("CONTROL9: ");
    //      Serial.println(CONTROL9);
    //      Serial.println();
    //      break;
    //    case 9:
    //      CONTROL10 = value;
    //        digitalWrite(R10,value);
    //      Serial.print("CONTROL10: ");
    //      Serial.println(CONTROL10);
    //
    //      Serial.println();
    //      break;
    //    case 10:
    //      CONTROL11 = value;
    //        digitalWrite(R11,value);
    //      Serial.print("CONTROL11: ");
    //      Serial.println(CONTROL11);
    //      Serial.println();
    //      break;
    //    case 11:
    //      CONTROL12 = value;
    //        digitalWrite(R12,value);
    //      Serial.print("CONTROL12: ");
    //      Serial.println(CONTROL12);
    //      Serial.println();
    //      break;
    //    case 12:
    //      CONTROL13 = value;
    //        digitalWrite(R13,value);
    //      Serial.print("CONTROL13: ");
    //      Serial.println(CONTROL13);
    //      Serial.println();
    //      break;
    //    case 13:
    //      CONTROL14 = value;
    //        digitalWrite(R14,value);
    //      Serial.print("CONTROL14: ");
    //      Serial.println(CONTROL14);
    //      Serial.println();
    //      break;
    //    case 14:
    //      CONTROL15 = value;
    //        digitalWrite(R15,value);
    //      Serial.print("CONTROL15: ");
    //      Serial.println(CONTROL15);
    //      Serial.println();
    //      break;
    //    case 15:
    //      CONTROL16 = value;
    //        digitalWrite(R16,value);
    //      Serial.print("CONTROL16: ");
    //      Serial.println(CONTROL16);
    //      Serial.println();
    //      break;
    case ERROR_VALUE:
      Serial.println("error");
      Serial.println();
      break;
    default:
      Serial.println("default");
      Serial.println();
  }

}
//*******************************

// Wifi credentials
const char *WIFI_SSID = "abdulloooh";
const char *WIFI_PASSWORD = "laptop124";

//Array to store 30 readings (and then transmit in one-go to AWS)
//short measurements[30];
//short measureIndex = 0;
unsigned long lastMeasurement = 0;
unsigned long timeFinishedSetup = 0;
unsigned long a = 0;

void writeEnergyToDisplay(double watts, double amps, double energy) {
  //  lcd.setCursor(3, 1); // Start from column 3, first two are broken :/

  Serial.print(amps);
  Serial.println("A");
  Serial.print(watts);
  Serial.println("W");
  cummulativeEnergy = cummulativeEnergy + energy;
  Serial.print("Cumulative Energy: ");
  Serial.print(cummulativeEnergy);
  Serial.println(" Wh");
  cummCost = cummulativeEnergy * UNITCOST / 1000;
  Serial.print("Cummulative cost: ");
  Serial.println(cummCost);
  sessionEnergy = cummulativeEnergy - refEnergy;
  Serial.print("Session Energy: ");
  Serial.println(sessionEnergy);
  sessionCost = sessionEnergy * UNITCOST / 1000;
  Serial.print("session cost: ");
  Serial.println(sessionCost);
  energyLeft = TARGET - sessionEnergy;
  Serial.print("Session Energy Left: ");
  Serial.println(energyLeft);
  percentUsage = (sessionEnergy * 100) / TARGET;
  Serial.print("Percentage session usage(%): ");
  Serial.println(percentUsage);
}

void printIPAddress() {
  //  lcd.setCursor(3,0);
  Serial.println("connected");
  Serial.println(WiFi.localIP());
}

void connectToWiFi() {
  //    Serial.clear();
  //  lcd.setCursor(3, 0);
  Serial.print("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-energy-monitor");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Only try 15 times to connect to the WiFi
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 10) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    delay(500);
    retries++;
  }

  // If we still couldn't connect to the WiFi, go to deep sleep for a
  // minute and try again.
  //    if(WiFi.status() != WL_CONNECTED){    Serial.print(".");
  //
  //        Serial.println("sleep mode");
  //        esp_sleep_enable_timer_wakeup(1 * 60L * 1000000L);
  //        esp_deep_sleep_start();
  //    }

  // If we get here, print the IP address on the LCD
  printIPAddress();
}

void setup()
{

  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(R4, OUTPUT);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  //set microcontroller resolution
  analogReadResolution(12);
  Serial.begin(115200);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsVClamp[thisReading] = 0;
    readingsGND[thisReading] = 0;
  }

connectToWiFi();

// set callibration
//  emon1.current(ADC_INPUT, 1);

//MQTT
client.setServer(mqttBroker, 1883); //httpserver,http port
client.setCallback(callback);
//  sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE_1);
//  client.subscribe(topicSubscribe);
//*********************************
timeFinishedSetup = millis();

}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  };
  Serial.print("Latency: ");
  Serial.print((millis() - a ) / 1000); //just to get latency
  Serial.println('s');
  a = millis();
  //  *****************************
  if (!client.connected()) {
    Serial.println("Attempting to reconnect");
    reconnect();
    // Subscribes for getting the value of the control variable in the temperature-box device
    char topicToSubscribe_variable_1[200];
    sprintf(topicToSubscribe_variable_1, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_1, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_1, "%s/%s/lv", topicToSubscribe_variable_1, VARIABLE_LABEL_SUB_1);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_1);
    client.subscribe(topicToSubscribe_variable_1);

    char topicToSubscribe_variable_2[200];
    sprintf(topicToSubscribe_variable_2, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_2, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_2, "%s/%s/lv", topicToSubscribe_variable_2, VARIABLE_LABEL_SUB_2);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_2);
    client.subscribe(topicToSubscribe_variable_2);

    char topicToSubscribe_variable_3[200];
    sprintf(topicToSubscribe_variable_3, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_3, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_3, "%s/%s/lv", topicToSubscribe_variable_3, VARIABLE_LABEL_SUB_3);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_3);
    client.subscribe(topicToSubscribe_variable_3);

    char topicToSubscribe_variable_4[200];
    sprintf(topicToSubscribe_variable_4, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_4, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_4, "%s/%s/lv", topicToSubscribe_variable_4, VARIABLE_LABEL_SUB_4);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_4);
    client.subscribe(topicToSubscribe_variable_4);

    char topicToSubscribe_variable_5[200];
    sprintf(topicToSubscribe_variable_5, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_5, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_5, "%s/%s/lv", topicToSubscribe_variable_5, VARIABLE_LABEL_SUB_5);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_5);
    client.subscribe(topicToSubscribe_variable_5);

    char topicToSubscribe_variable_6[200];
    sprintf(topicToSubscribe_variable_6, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_6, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_6, "%s/%s/lv", topicToSubscribe_variable_6, VARIABLE_LABEL_SUB_6);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_6);
    client.subscribe(topicToSubscribe_variable_6);

    char topicToSubscribe_variable_7[200];
    sprintf(topicToSubscribe_variable_7, "%s", ""); // Cleans the content of the char
    sprintf(topicToSubscribe_variable_7, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(topicToSubscribe_variable_7, "%s/%s/lv", topicToSubscribe_variable_7, VARIABLE_LABEL_SUB_7);
    //    Serial.println("subscribing to topic:");
    //    Serial.println(topicToSubscribe_variable_7);
    client.subscribe(topicToSubscribe_variable_7);
  }
  //********
  unsigned long currentMillis = millis();

  // If it's been longer then 1000ms since we took a measurement, take one now!
  double watt;
  double energy;
 double amps=0;
  if ((currentMillis - lastMeasurement) > 1000) {
   
    unsigned int i = 0;
    SumSqGND = 0;
    SumSqVClamp = 0;
    total = 0;

    for (unsigned int i = 0; i < numReadings; i++)
    {
      readingsVClamp[i] = analogRead(PinVClamp) - analogRead(PinVirtGND);
      delay(1); //
    }

    //Calculate Vrms
    for (unsigned int i = 0; i < numReadings; i++)
    {
      SumSqVClamp = SumSqVClamp + sq((float)readingsVClamp[i]);

    }

    total = sqrt(SumSqVClamp / numReadings);
    total = (total * (float)250 / 32768); // Rburden=3300 ohms, LBS= 0,004882 V (5/1024)
    // Transformer of 2000 laps (SCT-013-000).
    //using 12-bit ADC => 4096 and 3200ohms                      // 5*220*2000/(3300*1024)= 2/3 (aprox)

    //    Serial.println(total);
    //    delay(1500);
    amps = total;
    watt = amps * HOME_VOLTAG;
    //calc energy for the previous session
    if (timeCount > timeFinishedSetup) {
      Serial.println("Real time power reading");
      //send energy in Wh
      energy = (watt * (millis() - timeCount)) / (3.6 * pow(10, 6));
    }
    else {
      //send energy in Wh
      Serial.println("First Energy reading based on setup time");
      energy = (watt * (millis() - timeFinishedSetup)) / (3.6 * pow(10, 6));
    }
    timeCount = millis();
    lastMeasurement = millis();
    // Update the display
    writeEnergyToDisplay(watt, amps, energy);

  }
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  //  sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label
  //  ************************
  //   float sensor = hallRead();
  //  Serial.print("Value of Sensor is:- ");Serial.println(sensor);
  dtostrf(watt, 4, 2, str_power);
  dtostrf(cummulativeEnergy, 4, 2, str_energy);
  dtostrf(cummCost, 4, 2, str_cummcost);
  dtostrf(sessionCost, 4, 2, str_sessioncost);
  dtostrf(energyLeft, 4, 2, str_energyleft);
  dtostrf(sessionEnergy, 4, 2, str_sessionenergy);
  dtostrf(percentUsage, 4, 2, str_percentusage);
  //TESTING*****************
  // Important: Avoid to send a very long char as it is very memory space costly, send small char arrays
  sprintf(payload, "{\"");
  sprintf(payload, "%s%s\":%s", payload, VARIABLE_LABEL_1, str_power);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_2, str_energy);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_3, str_cummcost);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_5, str_sessioncost);
  sprintf(payload, "%s}", payload);
  //publish to Ubidots
  Serial.println("Publishing...");
  client.publish(topic, payload);

  /* Builds the payload with structure: {"energyleft":25.00,"percentageusage":50.00} for the rest of variables*/
  sprintf(payload, "%s", ""); //Cleans the content of the payload
  sprintf(payload, "{\"");
  sprintf(payload, "%s%s\":%s", payload, VARIABLE_LABEL_4, str_energyleft);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_6, str_percentusage);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_7, str_sessionenergy);
  sprintf(payload, "%s}", payload);
  //Publish to Ubidots
  Serial.println("Publishing...");
  client.publish(topic, payload);
  client.loop();
  delay(1000);
}

// Readings are unstable the first 5 seconds when the device powers on
//      // so ignore them until they stabilise.
//    if(millis() - timeFinishedSetup < 10000){
////      lcd.setCursor(3, 0);
//      Serial.println("Startup mode   ");
//    }else{
//      printIPAddress();
//      measurements[measureIndex] = watt;
//      measureIndex++;
//    }
