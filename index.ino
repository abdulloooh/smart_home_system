#include <Arduino.h>
#include <PubSubClient.h>
#include "EmonLib.h"
#include "WiFi.h"
#include <driver/adc.h>
// The GPIO pin were the CT sensor is connected to (should be an ADC input)
#define ADC_INPUT 34
#define HOME_VOLTAGE 220.0
// Force EmonLib to use 10bit ADC resolution
#define ADC_BITS    10
//define cummulative energy and its time
double timeCount = 0;
double cummulativeEnergy = 0;
// Create instances
EnergyMonitor emon1;
//*******************************
//MQTT
#define TOKEN "BBFF-4OMP5F0TB4SvzDCDMgRQVrePZBy6gR" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "abdulloooh" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
                                           //it should be a random and unique ascii string and different from all other devices
/****************************************
 * Define Constants
 ****************************************/
#define VARIABLE_LABEL_1 "power" // Assing the variable label
#define VARIABLE_LABEL_2 "energy" // Assing the variable label
#define VARIABLE_LABEL_SUB_1 "relay1" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_2 "relay2" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_3 "relay3" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_4 "relay4" // Assing the variable label to subscribe
#define VARIABLE_LABEL_SUB_5 "target" // Assing the variable label to subscribe
#define DEVICE_LABEL "esp32" // Assig the device label

#define R1 26 // Set the GPIO26 as RELAY
#define R2 27 // Set the GPIO27 as RELAY2
#define R3 28
#define R4 29

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[700];
char topic[1000];
//char topicSubscribe[100];
// Space to store values to send
char str_power[10];
char str_energy[10];

const int ERROR_VALUE = 65535;  // Set here an error value

const uint8_t NUMBER_OF_VARIABLES = 5; // Number of variable to subscribe to
char * variable_labels[NUMBER_OF_VARIABLES] = {"relay1", "relay2", "relay3", "relay4" , "target"}; // labels of the variable to subscribe to

float CONTROL1; // Name of the variable to be used within the code.
float CONTROL2; // Name of the variable to be used within the code.
float CONTROL3; // Name of the variable to be used within the code.
float CONTROL4; // Name of the variable to be used within the code.
float CONTROL5; // Name of the variable to be used within the code.

float value; // To store incoming value.
uint8_t variable; // To keep track of the state machine and be able to use the switch case.


/****************************************
   Initializate constructors for objects
 ****************************************/

WiFiClient ubidots;
PubSubClient client(ubidots);
/****************************************
 * Auxiliar Functions
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
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
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
      CONTROL1 = value;
      digitalWrite(R1,value);
      Serial.print("CONTROL1: ");
      Serial.println(CONTROL1);
      Serial.println();
      break;
    case 1:
      CONTROL2 = value;
        digitalWrite(R2,value);
      Serial.print("CONTROL2: ");
      Serial.println(CONTROL2);
      Serial.println();
      break;
    case 2:
      CONTROL3 = value;
        digitalWrite(R3,value);
      Serial.print("CONTROL3: ");
      Serial.println(CONTROL3);
      Serial.println();
      break;
    case 3:
      CONTROL4 = value;
        digitalWrite(R4,value);
      Serial.print("CONTROL4: ");
      Serial.println(CONTROL4);
      Serial.println();
      break;
    case 4:
      CONTROL5 = value;
//        digitalWrite(R5,value);
      Serial.print("CONTROL5: ");
      Serial.println(CONTROL5);
      Serial.println();
      break;
//    case 5:
//      CONTROL6 = value;
//        digitalWrite(R6,value);
//      Serial.print("CONTROL6: ");
//      Serial.println(CONTROL6);
//      Serial.println();
//      break;
//    case 6:
//      CONTROL7 = value;
//        digitalWrite(R7,value);
//      Serial.print("CONTROL7: ");
//      Serial.println(CONTROL7);
//
//      Serial.println();
//      break;
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
const char *WIFI_PASSWORD = "abdurrahman";

//Array to store 30 readings (and then transmit in one-go to AWS)
short measurements[30];
short measureIndex = 0;
unsigned long lastMeasurement = 0;
unsigned long timeFinishedSetup = 0;
int a;

void writeEnergyToDisplay(double watts, double amps, double energy){
//  lcd.setCursor(3, 1); // Start from column 3, first two are broken :/

  Serial.print(amps/60);
  Serial.println("A");
  Serial.print(watts);
  Serial.println("W");
  cummulativeEnergy = cummulativeEnergy + energy; 
  Serial.print(cummulativeEnergy);
  Serial.println(" kWh");
}

void printIPAddress(){
//  lcd.setCursor(3,0);
    Serial.println("connected");
    Serial.println(WiFi.localIP());
}

void connectToWiFi() {
//    Serial.clear();
//  lcd.setCursor(3, 0);
    Serial.print("WiFi...      ");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-energy-monitor");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Only try 15 times to connect to the WiFi
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 15){
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
  analogReadResolution(10);
  Serial.begin(115200);
//
//  lcd.init();
//  lcd.backlight();

  connectToWiFi();

  // Initialize emon library (30 = calibration number)
  emon1.current(ADC_INPUT, 30);

//  lcd.setCursor(3, 0);
//  lcd.print("AWS connect   ");
//  awsConnector.setup();
//

//*********************************
//MQTT
 client.setServer(mqttBroker, 1883); //httpserver,http port
 client.setCallback(callback);
//  sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE_1);
//  client.subscribe(topicSubscribe);
//*********************************
  timeFinishedSetup = millis();

}

void loop(){
  if(WiFi.status() != WL_CONNECTED){connectToWiFi();};
 Serial.print((millis() -a )/1000); //just to get latency
    Serial.println('s');
 a = millis();
//  *****************************
   if (!client.connected()) {
//    client.subscribe(topicSubscribe);
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
  }
//********
  unsigned long currentMillis = millis();

  // If it's been longer then 1000ms since we took a measurement, take one now!
  double watt;
  double energy;
  if((currentMillis - lastMeasurement) > 1000){
    double amps = 0;
    for(int i=0;i<60;i++){
      amps += emon1.calcIrms(1480); // Calculate Irms only
    }
    //average of 60 readings for current
      watt = (amps/60) * HOME_VOLTAGE;
      //calc energy for the previous session
      energy; //declare energy
      if(timeCount>timeFinishedSetup){
      energy = (watt * (millis()-timeCount))/(3.6 * pow(10,9));
        }
      else{
      energy = (watt * (millis()-timeFinishedSetup))/(3.6 * pow(10,9));
          }
      timeCount = millis();
    // Update the display
    lastMeasurement = millis();
    writeEnergyToDisplay(watt, amps, energy);

  }
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
//  sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label
//  ************************
//   float sensor = hallRead();
//  Serial.print("Value of Sensor is:- ");Serial.println(sensor);
  
  /* 4 is mininum width, 2 is precision; float value is copied onto str_power*/
  dtostrf(watt, 4, 2, str_power);
  dtostrf(cummulativeEnergy, 4, 2, str_energy);
//TESTING*****************
// Important: Avoid to send a very long char as it is very memory space costly, send small char arrays
  sprintf(payload, "{\"");
  sprintf(payload, "%s%s\":%s", payload, VARIABLE_LABEL_1, str_power);
  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_2, str_energy);
//  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_3, str_val_3);
//  sprintf(payload, "%s,\"%s\":%s", payload, VARIABLE_LABEL_4, str_val_4);
  sprintf(payload, "%s}", payload);
  
//  sprintf(payload, "%s {\"value\": %s}}", payload, str_power); // Adds the value
//  Serial.println("Publishing data to Ubidots Cloud");
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
//  }

  // When we have 30 measurements, send them to AWS!
//  if (measureIndex == 30) {
////    lcd.setCursor(3,0);
//    lcd.print("AWS upload..   ");
//
//    // Construct the JSON to send to AWS
//    String msg = "{\"readings\": [";
//
//    for (short i = 0; i <= 28; i++){
//      msg += measurements[i];
//      msg += ",";
//    }
//
//    msg += measurements[29];
//    msg += "]}";
//
//    awsConnector.sendMessage(msg);
//    measureIndex = 0;
//  }

  // This keeps the MQTT connection stable
//  awsConnector.loop();
