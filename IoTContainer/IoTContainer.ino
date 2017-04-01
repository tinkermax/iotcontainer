/* This is the ESP8266 Arduino firmware for the IoT Container electronics
 * as described at:
 * http://tinker.yeoman.com.au/2016/07/24/iot-container/
 *  
 * Malcolm Yeoman (2016) MIT License
 * 
 * Note: Wi-Fi username and password, MQTT Broker and IFTTT details
 *       need to be specified in associated Config.h file
 */
 
#include <ESP8266WiFi.h> 
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HX711.h>
#include <math.h>
#include <WiFiClientSecure.h>
#include "Config.h"

// Include API-Headers
extern "C" {
#include "user_interface.h" //***********
}

#define DEBUG false
//Comment out the following line to allow debugging info on serial monitor
#define Serial if(DEBUG)Serial

//Set Container ID
#define ContainerID "1" //Change this to be unique for each container!
#define mqttContainer "tinkermax/f/smartcontainer" ContainerID
#define mqttBatt "tinkermax/f/smartcontainerbatt" ContainerID
#define randName "Container" ContainerID

#define HX711_DATA 14 //GPIO14 (D5)
#define HX711_CLK 13 //GPIO13 (D7)
#define sensorDataWindow 3 //Number of rolling data points for noise rejection
#define RTC_MAGIC 0x75a78fc5
#define BATT_FULL 2812
#define BATT_CUTOFF 2280

//Specify container weight
//my units are grams (g), but any unit can be used by adjusting the scaleCalibration constant
const float containerEmptyWeight = 93; //Weight of the empty container
const float containerFullWeight = 2015; //Weight of the contents only (excluding container weight)
const float containerAlertWeight = 250; //Alert when contents reach this weight (excluding container weight)
const float scaleCalibration = 231.15; //manually adjust constant to match load cell reading to a known weight

//RTC memory structure
typedef struct {
  uint32 magic;
  uint64 calibration;
  float weight;
  boolean battwarningsent;
  boolean weightwarningsent;
} RTC;

HX711 scale(HX711_DATA, HX711_CLK, 128);

ADC_MODE(ADC_VCC);//Set the ADCC to read supply voltage.

RTC RTCvar;
WiFiClient wclient;
PubSubClient client(wclient);
float sensorData[sensorDataWindow+1];
float last_weight;
float battlevel;
boolean battwarningsent, weightwarningsent;
boolean connectionEstablished;

//All of the code is in setup, because there is no looping - only deep sleep between cycles
void setup() {
  boolean waiforOTA = false;
  char weight_str[20], battlevel_str[20];
  
  float weight, maxDeviation;
  int i, readingno, keeptrying;
  char buffer[100]; //buffer for the storing the sprintf outputs
  
  // Setup console
  Serial.begin(115200);
  Serial.println("Booting");

  //Read saved parameters
  system_rtc_mem_read(64, &RTCvar, sizeof(RTCvar));

  //Calibrate scale (manually adjust constant to match load cell to known weight)
  scale.set_scale(scaleCalibration); 

  // First time initialisation (after replacing battery and/or discharging caps to clear RTC memory)
  if (RTCvar.magic != RTC_MAGIC) {

    //Setup Scale
    scale.read();
    yield();
    scale.tare();               // reset the scale to 0 (with nothing on it)
    yield();
    last_weight = 0;
    
    RTCvar.magic = RTC_MAGIC;
    RTCvar.calibration = scale.get_offset();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    //Wi-Fi must be present on initialisation start up as a failure probably indicates incorrect credentials
    //In that case we do not want to go into deep sleep
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      Serial.print(".");
      delay(20000);
      ESP.restart();
    }
    
    Serial.println("");

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //Initialise OTA in case there is a software upgrade
    ArduinoOTA.setHostname("Container");
    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
      
    ArduinoOTA.begin();

    //wait 10 secs and check if button pressed, which will trigger a wait for the OTA reflash
    Serial.println("Waiting for any OTA updates");
    keeptrying = 20;
    while (keeptrying-- > 0 || waiforOTA == true) {
      if (digitalRead(0) == 0) {
        waiforOTA = true;
        Serial.print ("OTA");
      }
      Serial.print (".");
      ArduinoOTA.handle();
      delay(500);
    }

    Serial.println("");
   
    Serial.println("rtc memory init...");
    
  }
  
  // If waking up from deep sleep, lookup saved values
  else {
    scale.set_offset(RTCvar.calibration);
    last_weight = RTCvar.weight;
    battwarningsent = RTCvar.battwarningsent;
    weightwarningsent = RTCvar.weightwarningsent;
  }

  //specify mqtt server
  client.setServer(mqtt_server, 1883);

  //Init Sensor values for rolling data window
  for (i = 0; i < sensorDataWindow; i++) {
    sensorData[i] = 0;
  }

  //wait until all readings have agreed to within 3 grams
  do {
    //Read weight
    weight = scale.get_units();
    //Serial.println (weight);
  
    if (weight < 0) {
      weight = 0;
    }
    
    maxDeviation = 0;
    sensorData[sensorDataWindow] = weight;

    for (i = 0; i < sensorDataWindow; i++) {
      sensorData[i] = sensorData[i+1];
      if (abs(weight - sensorData[i]) > maxDeviation) {
        maxDeviation = abs(weight - sensorData[i]);
      }
    }

  } while (maxDeviation > 3);

  //Serial.print("Reading weight:");
  //Serial.println (weight);

  //Read supply voltage
  //Need to recharge at BATT_CUTOFF (represents 3.6V before diode)
  battlevel= ESP.getVcc();

  //reset battery warning if not low (allowing for hysteresis)
  if (battlevel > BATT_CUTOFF + 50) {
    battwarningsent = false;
  }
    
  //reset weight warning if not low (allowing for hysteresis)
  if (weight > containerAlertWeight + containerEmptyWeight + 50) {
    weightwarningsent = false;
  }
    
  //only update if weight has changed by more than 10g, and the container is on the scale
  if ((abs(weight - last_weight) >= 10 && weight >= containerEmptyWeight)
      || (battlevel <= BATT_CUTOFF && battwarningsent == false))         //or low batt warning
  {
    
    for (i = 0; i < sensorDataWindow; i++) {
      sensorData[i] = weight;
    }

    //reconnect wifi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print ("Starting Wifi");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
    }

    keeptrying = 30;
    while ((WiFi.status() != WL_CONNECTED) && keeptrying-- > 0) {
      Serial.print (".");
      delay (1000);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      
      Serial.println ("Wifi started");
      
      //Reconnect to MQTT broker
      client.loop();
      if (!client.connected()) {
        Serial.println("Reconnecting to MQTT Broker");
        connectionEstablished = reconnect();
      }

      // if successfully connected to broker, proceed to send message
      if (connectionEstablished == true) {
        Serial.println("Sending mqtt updates"); 
         
        //Calc weight in terms of %full
        dtostrf(100.0 * (weight - containerEmptyWeight) / containerFullWeight,1,0,weight_str);
        //Calc batt in terms of %full
        dtostrf(100.0 * (battlevel - BATT_CUTOFF) / (BATT_FULL - BATT_CUTOFF),1,0,battlevel_str);
        client.publish(mqttContainer, weight_str);
        client.publish(mqttBatt, battlevel_str);
        //sprintf(buffer, "%s,%s", weight_str, battlevel_str);
        //Serial.println(buffer);
        
        //Only send one battery warning
        if (battlevel <= BATT_CUTOFF) {
          sprintf(buffer, "Please replace Greek yoghurt battery - only %s%% charge left", battlevel_str);
          if (sendNotification(buffer) > 0) {
            battwarningsent = true;
          }
        }
  
        //Only send one replenishment warning
        if (weight <= (containerAlertWeight + containerEmptyWeight) && weightwarningsent == false) {
          sprintf(buffer, "Please replenish Greek yoghurt - only %s%% remaining", weight_str);
          if (sendNotification(buffer) > 0) {
            weightwarningsent = true;
          }
        }
  
        last_weight = weight;
      }
    }
  }
    
  RTCvar.weight = last_weight;
  RTCvar.battwarningsent = battwarningsent;
  RTCvar.weightwarningsent = weightwarningsent;
  system_rtc_mem_write(64, &RTCvar, sizeof(RTCvar));
  delay(1);
  
  //Sleep for 60 minutes
  ESP.deepSleep(60 * 60 * 1000000, WAKE_RF_DEFAULT);
  delay(500);

}

boolean reconnect() {
  // Loop until we're reconnected to MQTT Broker - but try a maximum of 5 times to avoid draining battery
  int keeptrying = 5;
  
  while (!client.connected() && keeptrying-- > 0) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(randName, mqttuser, mqttpwd)) {
      Serial.println("connected");
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
  return false;
}

int sendNotification(char *messageToSend) {
  WiFiClientSecure httpspost;
  char post_rqst[256];
  char *p = post_rqst;
  char payload[30];
  char *json = payload;
  char jsonlen[2];
  
  if (!httpspost.connect("maker.ifttt.com", 443)) {
    Serial.println("Connection failed");
    return 0;
  }

  strcpy(json, "{\"value1\":\"");
  strcat(json, messageToSend);
  strcat(json, "\"}");
  itoa(strlen(json),jsonlen,10);

  strcpy(p, "POST ");
  strcat(p, iftttMakerUrl);
  strcat(p, " HTTP/1.1\r\n");
  strcat(p, "Host: maker.ifttt.com\r\n");
  strcat(p, "Content-Type: application/json\r\n");
  strcat(p, "Content-Length: ");
  strcat(p, jsonlen);
  strcat(p, "\r\n\r\n");
  strcat(p, json);
  strcat(p, "\r\n\r\n");
  
  httpspost.print(post_rqst); 

  //Check post was successfully received
  while (httpspost.connected()) {
    String line = httpspost.readStringUntil('\n');
    Serial.println(line);
    if (line.indexOf("200 OK") > 0) {
      Serial.println(">>Success!");
      httpspost.stop(); 
      return 1;
    }
  }

  httpspost.stop(); 
  return 1; 
}

void loop() {
}
