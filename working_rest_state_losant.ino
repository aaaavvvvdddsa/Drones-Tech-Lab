#define TINY_GSM_MODEM_SIM7600
#define LOGGING true

// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

#include <ArduinoJson.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1



// // Define how you're planning to connect to the internet
// // These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false

// Your GPRS credentials, if any
const char apn[] = "JioNet";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* LOSANT_DEVICE_ID = "6489798a66eafd67a168f2be";
const char* LOSANT_ACCESS_KEY = "59d51f39-7fc6-405d-b32f-735a1725827e";                                 // change
const char* LOSANT_ACCESS_SECRET = "415c28f3af9338935a6a431f5d6a66590cd17d81cd50fae7c9295eaf38d9bad9";  //change
String token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI2NDg5NzkyNzIxMjJmY2NmMGI5NmEwMTAiLCJzdWJ0eXBlIjoiYXBpVG9rZW4iLCJzY29wZSI6WyJhbGwuQXBwbGljYXRpb24iXSwiaWF0IjoxNjg2NzMxMDQ3LCJpc3MiOiJhcGkuZ2V0c3RydWN0dXJlLmlvIn0.lc7kUcC0xk0_A6Vl0Q8q_oy9UxUho9acKAQPcRMdffc";

// Server details
const char server[] = "api.losant.com";
const char resourceAuth[] = "/auth/device";
const char resourceState[] = "/applications/646dc0732a381fe51a416d76/devices/6489798a66eafd67a168f2be/state";
const int port = 80;

bool needAuth = true;
bool sendHTTPRequest(String resource, String payload);

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
HttpClient http(client, server, port);

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(500);

  SerialMon.println("Wait...");
  // Set GSM module baud rate
  // TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  SerialAT.begin(115200);
  delay(1000);
  connectToNetwork();
  connectToInternet();
}

void connectToNetwork() {
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  delay(5000);
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.print("Network not Found!");
    delay(5000);
    return;
  }
  SerialMon.println(" Success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  } else connectToNetwork();
}

void connectToInternet() {

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" Failed");
    delay(5000);
    return;
  }
  SerialMon.println(" Success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  } else connectToInternet();
}


void loop() {

  if (!modem.isGprsConnected() && modem.isNetworkConnected()) { connectToInternet(); }
  if (!modem.isNetworkConnected()) {
    connectToNetwork();
    connectToInternet();
  }

  // while (needAuth) sendAuth();
  // sendAuth();
  // delay(5000);
  sendState();
  delay(5000);

  // Do nothing forevermore
  // while (true) { delay(1000); }
}

bool sendAuth() {

  DynamicJsonDocument jsonDocument(200);
  JsonObject root = jsonDocument.to<JsonObject>();
  root["deviceId"] = LOSANT_DEVICE_ID;
  root["key"] = LOSANT_ACCESS_KEY;
  root["secret"] = LOSANT_ACCESS_SECRET;


  String authPayload;
  serializeJson(root, authPayload);
  Serial.println("Authenticating.....");
  sendHTTPRequest(resourceAuth, authPayload);
}

bool sendState() {

  DynamicJsonDocument jsonDocument(200);
  JsonObject root = jsonDocument.to<JsonObject>();

  root["data"]["temp"] = 33.874;

  String Statepayload;
  serializeJson(root, Statepayload);

  Serial.println("Sending Device State....");

  sendHTTPRequest(resourceState, Statepayload);
}



bool sendHTTPRequest(String resource, String payload) {

  SerialMon.println(F("Performing HTTP POST request... "));
  http.beginRequest();
  int err = http.post(resource);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", payload.length());
  http.sendHeader("Authorization", "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI2NDg5NzkyNzIxMjJmY2NmMGI5NmEwMTAiLCJzdWJ0eXBlIjoiYXBpVG9rZW4iLCJzY29wZSI6WyJhbGwuQXBwbGljYXRpb24iXSwiaWF0IjoxNjg2NzMxMDQ3LCJpc3MiOiJhcGkuZ2V0c3RydWN0dXJlLmlvIn0.lc7kUcC0xk0_A6Vl0Q8q_oy9UxUho9acKAQPcRMdffc");
  http.beginBody();
  http.print(payload);
  http.endRequest();

  if (err != 0) {
    SerialMon.println(F("HTTP request Failed"));
    delay(5000);
    // return;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(5000);
    return;
  }

  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    SerialMon.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
    SerialMon.print(F("Content length is: "));
    SerialMon.println(length);
  }

  if (http.isResponseChunked()) {
    SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  // if (status == 200) needAuth = false;

  // Shutdown
  // http.stop();
  // SerialMon.println(F("Server disconnected"));
}
