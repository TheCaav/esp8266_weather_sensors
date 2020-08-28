#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
//#include <Adafruit_AHT10.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// BME SENSOR
/** Needed for Spi
#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/
#define SOIL_MOISTURE_PIN A0

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//Adafruit_AHT10 aht;
//Adafruit_Sensor *aht_humidity, *aht_temp;

long delayTime;

WiFiUDP udp;
IPAddress host(10, 3, 141, 1);
uint16_t port = 8125;
const char *ssid = "raspi-webgui";
const char *password = "PLACEHOLDER";

void printBMEValues()
{
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    udp.beginPacket(host, port);
    String message = "temp,place=grow,sensor=bme280:" + String(temp_event.temperature) + "|g";
    char chars[50];
    message.toCharArray(chars, 40, 0);
    udp.write(chars);
    Serial.println(chars);
    udp.endPacket();

    udp.beginPacket(host, port);
    message = "humi,place=grow,sensor=bme280:" + String(humidity_event.relative_humidity) + "|g";
    chars[50];
    message.toCharArray(chars, 40, 0);
    udp.write(chars);
    Serial.println(chars);
    udp.endPacket();

    udp.beginPacket(host, port);
    message = "pres,place=grow,sensor=bme280:" + String(pressure_event.pressure) + "|g";
    char chars2[50];
    message.toCharArray(chars2, 40, 0);
    udp.write(chars2);
    Serial.println(chars2);
    udp.endPacket();

    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");

    Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(" %");

    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    Serial.println();
}

/**void printAHTValues()
{
    sensors_event_t humidity;
    sensors_event_t temp;
    aht_humidity->getEvent(&humidity);
    aht_temp->getEvent(&temp);

    udp.beginPacket(host, port);
    String message = "temp,place=grow,sensor=aht10:" + String(temp.temperature) + "|g";
    char chars[38];
    message.toCharArray(chars, 38, 0);
    udp.write(chars);
    Serial.println(chars);
    udp.endPacket();

    udp.beginPacket(host, port);
    message = "humi,place=grow,sensor=aht10:" + String(humidity.relative_humidity) + "|g";
    chars[38];
    message.toCharArray(chars, 38, 0);
    udp.write(chars);
    Serial.println(chars);
    udp.endPacket();

    Serial.println("AHT10 Readings:");

    Serial.print("\t\tHumidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println(" % rH");
    Serial.print("\t\tTemperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");
}*/

void printSoilMoisture() {
    int val = analogRead(SOIL_MOISTURE_PIN);

    udp.beginPacket(host, port);
    String message = "soil_moisture,place=grow,sensor=capacitative:" + String(val) + "|g";
    char chars[60];
    message.toCharArray(chars, 60, 0);
    udp.write(chars);
    Serial.println(chars);
    udp.endPacket();

    Serial.println("Moisture Reading:");

    Serial.print("\t\tSoil Moisture: ");Serial.print(val);Serial.println(" Potatos");
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Hello world");
    //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    if (!bme.begin(0x76))
    {
        Serial.println("Could not find BME280 Sensor!");
    }
    else
    {
        Serial.println("-- Default Test BME280 --");
        Serial.println("normal mode, 16x oversampling for all, filter off,");
        Serial.println("0.5ms standby period");
        bme_temp->printSensorDetails();
        bme_pressure->printSensorDetails();
        bme_humidity->printSensorDetails();
    }
    /**if (!aht.begin()) {
        Serial.println("Could not find AHT10!");
    } else {
        Serial.println("AHT found");
        aht_temp = aht.getTemperatureSensor();
        aht_temp->printSensorDetails();

        aht_humidity = aht.getHumiditySensor();
        aht_humidity->printSensorDetails();
    }*/

    delayTime = 5000;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    WiFi.setAutoReconnect(true);

    Serial.println("");
    Serial.println("Wifi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("All great: Begin taking measurements");
}

void loop()
{
    printBMEValues();
    printSoilMoisture();
    //printAHTValues();
    delay(delayTime);
}
