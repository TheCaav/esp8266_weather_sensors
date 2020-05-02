#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Adafruit_AHT10.h>

// BME SENSOR
#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

Adafruit_AHT10 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;

long delayTime;

void printBMEValues() {
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

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

void printAHTValues () {
    sensors_event_t humidity;
    sensors_event_t temp;
    aht_humidity->getEvent(&humidity);
    aht_temp->getEvent(&temp);

    Serial.println("AHT10 Readings:");

    Serial.print("\t\tHumidity: ");Serial.print(humidity.relative_humidity);Serial.println(" % rH");
    Serial.print("\t\tTemperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
}

void setup() {
    Serial.begin(9600);
    Serial.println("helllo world");
    //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));


    if (!bme.begin(0x76)) {
        Serial.println("Could not find BME280 Sensor!");
    }
    Serial.println("-- Default Test BME280 --");
    Serial.println("normal mode, 16x oversampling for all, filter off,");
    Serial.println("0.5ms standby period");
    bme_temp->printSensorDetails();
    bme_pressure->printSensorDetails();
    bme_humidity->printSensorDetails();
    
    if (!aht.begin()) {
        Serial.println("Could not find AHT10!");
    } else {
        Serial.println("AHT found");
        aht_temp = aht.getTemperatureSensor();
        aht_temp->printSensorDetails();

        aht_humidity = aht.getHumiditySensor();
        aht_humidity->printSensorDetails();
    }
    
    delayTime = 5000;

    Serial.println("All great: Begin taking measurements");
}

void loop() {
    printBMEValues();
    printAHTValues();
    delay(delayTime);
}
