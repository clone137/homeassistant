STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));                  // select the u.FL antenna

/*
    Currently installed on "BEDROOM"

	Wiring
	------
    PIR Vcc to PHOTON pin 3v3
	PIR GND to PHOTON pin GND
	PIR OUT to PHOTON pin D2
	
	PIR LED + to PHOTON pin D0
	PIR LED - to PHOTON pin GND

    MQTT LED + to PHOTON pin D4
    MQTT LED - to PHOTON pin GND

	DS18B20 Vcc to PHOTON pin 3v3
	DS18B20 GND to PHOTON pin GND
	DS18B20 OUT to PHOTON pin D3
	
	LDR Vcc to PHOTON pin 3v3
	LDR GND to PHOTON pin GND
	LDR A0 to PHOTON pin A0

    DHT22 pin 1 to PHOTON pin +5V
	DHT22 pin 2 to PHOTON pin DHTPIN
	DHT22 pin 4 to PHOTON pin GND
	10k resistor between pin 1 (+5V) and pin 2 (data)
*/

#include "MQTT.h"
#include <OneWire.h>
#include <DS18.h>
#include <math.h>
#include <PietteTech_DHT.h>
#include <SparkJson.h>

// Timers
uint32_t        now;
uint32_t        msLastMetric;
uint32_t        msLastSample;
uint32_t        mqttLastMetric;
const uint32_t  msSAMPLE_INTERVAL   = 5000;
const uint32_t  msMETRIC_PUBLISH    = 10000;
const uint32_t  ms_INTERVAL         = 15000;
const uint32_t  mqtt_INTERVAL       = 60000;

// MQTT
void callback(char* topic, byte* payload, unsigned int length);
byte            MQTTbroker[]        = { 192,168,1,70 };     //  MQTT broker IP
MQTT            client(MQTTbroker, 1883, callback);
int             mqttLedPin          = D4;                   // LED Pin

// PIR variables
int             pirInputPin         = D2;                   // choose the input pin (for PIR sensor)
int             pirLedPin           = D0;                   // LED Pin
int             pirState            = LOW;                  // we start, assuming no motion detected
int             pirVal              = 0;                    // variable for reading the pin status
int             calibrateTime       = 5000;                 // wait for the thingy to calibrate

// Commented out for now while testing AM2302
// DS18B20 variables
char            ds18b20Info[64];
double          celsius;
const int       MAXRETRY            = 4;
DS18            ds18b20(D3);

// LDR variables
int             ldrPin              = A0;
double          ldrReading;
char            ldrInfo[64];

// DHT22/AM2302 variables
#define         DHTTYPE             DHT22                   // Sensor type DHT11/21/22/AM2301/AM2302
#define         DHTPIN              D1                      // Digital pin for communications
PietteTech_DHT  DHT(DHTPIN, DHTTYPE);
double          dht22Reading;
char            dht22Info[64];

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);

  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(p);
  
  const char* name = root["name"];                          // JSON object 'name'

  if (!strcmp(name, "frontdoor")) {
    setMQTTLED(HIGH);
    Serial.println(name);
    mqttLastMetric = millis();
  }
}

void setup() {
  // setup the PIR
  pinMode(pirLedPin, OUTPUT);
  pinMode(mqttLedPin, OUTPUT);
  pinMode(pirInputPin, INPUT);                              // declare sensor as input

  // setup the DS18B20
  Particle.variable("temperature", &celsius, DOUBLE);

  // setup the LDR
  pinMode(ldrPin, INPUT);
  Particle.variable("lightlevel", &ldrReading, DOUBLE);

  // setup the DHT22/AM2302
  DHT.begin();

  // setup MQTT
  client.connect(System.deviceID());
  if (client.isConnected()) {
    client.publish("stat/bedroom", "startup");
    client.subscribe("kios/mqtt");
  }
}

void loop() {
  // MQTT
  if (client.isConnected())
    client.loop();
  else
    client.connect(System.deviceID());

  client.subscribe("kios/mqtt");
  // PIR
  // if the sensor is calibrated
  if (calibrated()) {
    // get the data from the sensor
    readPir();
    // report it out, if the state has changed
    publishPirData();
  }

  // DS18B20 && LDR && DHT22/AM2302
  now = millis();
  if (now - msLastSample >= ms_INTERVAL) {
    if (ds18b20.read()) {
      celsius = ds18b20.celsius();
    } else {
      return;
    }

    DHT.acquireAndWait(1000);               // wait up to 1 sec (default indefinitely)
    ldrReading  = analogRead (ldrPin);
    msLastSample = millis();
    publishDs18b20Data();
    publishLdrData();
    publishDht22Data();
    msLastMetric = millis();
  }
  if (now - mqttLastMetric >= mqtt_INTERVAL) {
    setMQTTLED(LOW);
  }
}

void readPir() {
  pirVal = digitalRead(pirInputPin);
}

bool calibrated() {
  return millis() - calibrateTime > 0;
}

void setPIRLED(int state) {
  digitalWrite(pirLedPin, state);
}

void setMQTTLED(int state) {
  digitalWrite(mqttLedPin, state);
}

void flashLED() {
  digitalWrite(pirLedPin, HIGH);
  digitalWrite(pirLedPin, LOW);
  digitalWrite(pirLedPin, HIGH);
  digitalWrite(pirLedPin, LOW);
  digitalWrite(pirLedPin, HIGH);
  digitalWrite(pirLedPin, LOW);
  digitalWrite(pirLedPin, HIGH);
  digitalWrite(pirLedPin, LOW);
  digitalWrite(pirLedPin, HIGH);
  digitalWrite(pirLedPin, LOW);
  digitalWrite(pirLedPin, HIGH);
}

void publishPirData() {
  if (pirVal == HIGH) {
    // the current state is no motion
    // i.e. it's just changed
    // announce this change by publishing an event
    if (pirState == LOW) {
      // we have just turned on
      Particle.publish("PIR", "motion", PRIVATE);
      client.publish("stat/bedroom/PIR", "motion");
      // Update the current state
      pirState = HIGH;
      setPIRLED(pirState);
    }
  } else {
    if (pirState == HIGH) {
      // we have just turned of
      // Update the current state
      Particle.publish("PIR", "no motion", PRIVATE);
      client.publish("stat/bedroom/PIR", "still");
      pirState = LOW;
      setPIRLED(pirState);
    }
  }
}

void publishDs18b20Data() {
  sprintf(ds18b20Info, "%2.1f", celsius);
  Particle.publish("temperature", ds18b20Info, PRIVATE);
  client.publish("stat/bedroom/TEMPERATURE", ds18b20Info);
}

void publishLdrData() {
  sprintf(ldrInfo, "%2.0f", ldrReading);
  Particle.publish("lightlevel", ldrInfo, PRIVATE);
  client.publish("stat/bedroom/LIGHTLEVEL", ldrInfo);
}

void publishDht22Data() {
  dht22Reading = DHT.getHumidity();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22Humidity", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22HUMIDITY", dht22Info);

  dht22Reading = DHT.getCelsius();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22Celsius", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22CELSIUS", dht22Info);

  dht22Reading = DHT.getFahrenheit();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22Fahrenheit", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22FAHRENHEIT", dht22Info);

  dht22Reading = DHT.getKelvin();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22Kelvin", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22KELVIN", dht22Info);

  dht22Reading = DHT.getDewPoint();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22DewPoint", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22DEWPOINT", dht22Info);

  dht22Reading = DHT.getDewPointSlow();
  sprintf(dht22Info, "%2.1f", dht22Reading);
  Particle.publish("dht22DewPointSlow", dht22Info, PRIVATE);
  client.publish("stat/bedroom/DHT22DEWPOINTSLOW", dht22Info);
}
