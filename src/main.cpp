#include <Arduino.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#include <SensirionI2CScd4x.h>
#include <Wire.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ESP8266Ping.h>

#include <ctime>  // Ajout de la bibliothèque TimeLib.h

SensirionI2CScd4x scd4x;

// SCD4x
const int16_t SCD_ADDRESS = 0x62;

const char* ssid = "Livebox-B780";
const char* password = "NkQeVz54GNRnDUMQYu";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
unsigned long timerDelay = 30 * 1000; // 5 minutes * 60

//Your Domain name with URL path or IP address with path
String serverName = "http://192.168.1.41:8083/sensors/";

// calculate CRC according to datasheet section 5.17
uint8_t CalcCrc(uint8_t data[2]) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < 2; i++) {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

void printSensorData(float co2, float temperature, float humidity) {
  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.print("\tTemperature: ");
  Serial.print(temperature);
  Serial.print("\tHumidity: ");
  Serial.println(humidity);
}

String toJson(const char* co2, const char* temperature, const char* humidity) {
  DynamicJsonDocument doc(256);
  doc["co2"] = co2;
  doc["temp"] = temperature;
  doc["humidity"] = humidity;

  String json;
  serializeJson(doc, json);
  return json;
}

void requestPOST(String jsonData, HTTPClient& http, WiFiClient& client, String serverName) {
  Serial.println("Sending HTTP request...");

  // You can add these debug lines to check the connection status before making the request
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());

  Serial.print("Connecting to server: ");
  Serial.println(serverName);

  http.begin(client, serverName);

  Serial.println(jsonData);
  Serial.print("\n");

  // If you need Node-RED/server authentication, insert user and password below
  // http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");

  // Specify content-type header
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");

  Serial.println("Trying to connect to the server...");
  int httpResponseCode = http.POST(jsonData);
  Serial.println("HTTP POST request sent...");

  if (httpResponseCode != HTTP_CODE_OK) {
    Serial.print("HTTP POST failed with code ");
    Serial.println(httpResponseCode);
    Serial.print("HTTP Error: ");
    Serial.println(http.errorToString(httpResponseCode).c_str());
    String serverResponse = http.getString();
    Serial.print("Server response: ");
    Serial.println(serverResponse);
  } else {
    Serial.println("HTTP POST successful");
  }

  http.end();
}

unsigned long lastRecalibrationTime = 0;
const unsigned long recalibrationInterval = 60 * 60 * 1000; // 1 hour.

void performRecalibration() {
  Serial.println("# Performing recalibration");

  // Assuming an external reference shows 650 ppm
  uint16_t calibration = 650;

  // Prepare buffer with data for calibration
  uint8_t data[3];
  data[0] = (calibration & 0xFF00) >> 8;
  data[1] = calibration & 0x00FF;
  data[2] = CalcCrc(data);

  // Send command for forced recalibration
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x36);
  Wire.write(0x2F);
  // Append data for calibration (2 bytes calibration, 1 byte CRC)
  Wire.write(data, 3);
  Wire.endTransmission();

  delay(400);

  // Read data: 2 bytes correction, 1 byte CRC
  Wire.requestFrom(SCD_ADDRESS, 3);
  uint8_t counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  if (CalcCrc(data) != data[2])
    Serial.println("# ERROR: Recalibration CRC return value");

  calibration = ((uint16_t)data[0] << 8 | data[1]);

  Serial.print("# Value after recalibration\n# ");
  Serial.println(calibration - 32768);

  // Output format
  Serial.println("CO2(ppm)\tTemperature(degC)\tRelativeHumidity(percent)");

  // Start SCD measurement again in periodic mode, will update every 2 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();

  // Wait for first measurement to be finished (> 5 s)
  delay(10 * 1000);
}

void setup() {
  Serial.begin(115200);

  // Init I2C
  Wire.begin();

  // Wait until sensors are ready, > 1000 ms according to datasheet
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Essayez de pinger le serveur
    if (Ping.ping("192.168.1.41")) {
      Serial.println("Ping success!");
    } else {
      Serial.println("Ping failed!");
    }

    // Reste du code...
  }

  WiFiClient client;
  HTTPClient http;

  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Timer set to 5 minutes (timerDelay variable), it will take 5 minutes before publishing the first reading.");

  // Start SCD measurement in periodic mode, will update every 5 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();

  // Wait for 5 minutes to equilibrate sensor to ambient
  Serial.println("# Waiting 5 minutes for equilibration");
  delay(5 * 60 *  1000); //

  Serial.println("# CO2 values before calibration");

  // Measure 5 times
  for (uint8_t repetition = 0; repetition < 5; repetition++) {
    // Read measurement data
    Wire.requestFrom(SCD_ADDRESS, 12);
    uint8_t counter = 0;
    uint8_t data[12];
    while (Wire.available()) {
      data[counter++] = Wire.read();
    }

    // Floating-point conversion according to datasheet
    float co2 = (float)((uint16_t)data[0] << 8 | data[1]);
    float temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
    float humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

    printSensorData(co2, temperature, humidity);

    delay(2000);
  }

  // Stop SCD measurement
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x3f);
  Wire.write(0x86);
  uint8_t ret = Wire.endTransmission();
  Serial.println(ret);

  // Wait for the sensor
  delay(20);

  // Perform initial calibration
  performRecalibration();

  // Wait for sensor
  delay(20);

  // Start SCD measurement again in periodic mode, will update every 2 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();

  // Wait for first measurement to be finished (> 5 s)
  delay(5000);
}

void loop() {
  // Send read data command
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0xec);
  Wire.write(0x05);
  Wire.endTransmission();

  WiFiClient client;
  HTTPClient http;

  // http.addHeader("Connexion", "Keep-Alive");

  // Read measurement data
  Wire.requestFrom(SCD_ADDRESS, 12);
  uint8_t counter = 0;
  uint8_t data[12];
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Floating-point conversion according to datasheet
  float co2 = (float)((uint16_t)data[0] << 8 | data[1]);
  float temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
  float humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

  // Convertir les float en chaînes de caractères
  char co2Str[5];
  char temperatureStr[6];  // 5 caractères au total, 1 décimale
  char humidityStr[4];

  // Convertir co2 sans décimale
  itoa((int)co2, co2Str, 10);

  // Convertir temperature avec 1 décimale
  dtostrf(temperature, 4, 1, temperatureStr);

  // Convertir humidity sans décimale
  itoa((int)humidity, humidityStr, 10);

  printSensorData(co2, temperature, humidity);

  if ((millis() - lastTime) > timerDelay) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
      printSensorData(co2, temperature, humidity);
      requestPOST(toJson(co2Str, temperatureStr, humidityStr), http, client, serverName);
    } else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }

  // Wait 30 seconds for the next measurement
  delay(30 * 1000);

  // Check if it's time for recalibration
  if (millis() - lastRecalibrationTime >= recalibrationInterval || co2 > 2000 || co2 < 400) {
    performRecalibration();
    lastRecalibrationTime = millis();
  }
}
