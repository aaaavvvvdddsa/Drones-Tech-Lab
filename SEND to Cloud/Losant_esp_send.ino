#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Losant.h>

SoftwareSerial nodemcu(12, 14);  // 12(D6)(rx-master) => tx(slave),  14(D5)(tx-mater) => rx(slave)
WiFiClient wifiClient;
float temperature = 25.0;

// WiFi credentials.
const char* WIFI_SSID = "DTL_Jio_4G";
const char* WIFI_PASS = "Drones_777";

// Losant credentials.
const char* LOSANT_DEVICE_ID = "646e13a22a381fe51a416e0c";
const char* LOSANT_ACCESS_KEY = "3c2cb95a-6715-4884-8aab-9c0843986be3";
const char* LOSANT_ACCESS_SECRET = "524aae053e405f5477d93c4cda674a537de710ca288bac98c4cc973b9f63dcf7";

LosantDevice device(LOSANT_DEVICE_ID);

void connect() {

  // Connect to Wifi.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to Losant.
  Serial.println();
  Serial.print("Connecting to Losant...");

  // device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  device.connect(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  while (!device.connected()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected!");
}

void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  while (!Serial) continue;

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  connect();
  nodemcu.begin(9600);
}

void push() {



  // Check if the other Arduino is transmitting
  if (nodemcu.available()) {
    

    // Allocate the JSON document
    // This one must be bigger than the sender's because it must store the strings
    StaticJsonDocument<1000> doc;

    // Read the JSON document from the "link" serial port
    DeserializationError serial = deserializeJson(doc, nodemcu);

    if (serial == DeserializationError::Ok) {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      // Serial.print("timestamp = ");
      // Serial.println(doc["methane"].as<float>());
      // // Serial.print("value = ");
      // Serial.println(doc["pm10"].as<float>());
         
    // Losant uses a JSON protocol. Construct the simple state object.
    StaticJsonDocument<1000> jsonBuffer;
    JsonObject root = jsonBuffer.to<JsonObject>();
    // Serial.println(root["temp"]);
    root["temp"] = doc["temp"];
    root["TDS"] = doc["tds"];
    root["pH"] = doc["ph"];
    root["EC"] = doc["ec"];
    root["ORP"] = doc["orp"];
    root["DO"] = doc["do"];
    root["turbidity"] = doc["turbidity"];

    root["methane"] = doc["methane"];
    root["pm10"] = doc["pm10"];
    root["pm1"] = doc["pm1"];
    root["pm2_5"] = doc["pm2_5"];
    root["geiger"] = doc["geiger"];
    root["ethanol"] = doc["ethanol"];
    root["methane2"] = doc["methane2"];
    root["hydrogen"] = doc["hydrogen"];
    root["ammonia"] = doc["ammonia"];
    root["co"] = doc["co"];
    root["no2"] = doc["no2"];

    // // Send the state to Losant.
    device.sendState(root);
    } else {
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(serial.c_str());

      // Flush all bytes in the "link" serial port buffer
      while (nodemcu.available() > 0)
        nodemcu.read();
    }
  }
  // delay(2000);
}

void loop() {

  bool toReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }

  if (!device.connected()) {
    Serial.println("Disconnected from Losant");
    toReconnect = true;
  }

  if (toReconnect) {
    connect();
  }
  push();
  device.loop();
}