#include <Wire.h>                // Include the Wire library for I2C
#include <LiquidCrystal_I2C.h>    // Include the LiquidCrystal_I2C library for the I2C LCD
#include "DHT.h"

#define DHTPIN 2                 // Pin connected to the data pin of the DHT sensor
#define DHTTYPE DHT11            // Define the sensor type as DHT11
#define FAN_RELAY_PIN 3          // Pin connected to the relay module for the fan
#define HUMIDIFIER_PIN 9         // Pin connected to the relay module for the humidifier
#define WATER_SENSOR_PIN A0      // Pin connected to the water level sensor
#define PUMP_RELAY_PIN 7         // Pin connected to the relay module for the water pump

// Set the LCD address to 0x27 (adjust if necessary), 16 characters, and 2 lines
LiquidCrystal_I2C lcd(0x27, 16, 2);

DHT dht(DHTPIN, DHTTYPE);
bool fanStatus = false;
bool humidifierStatus = false;
bool pumpStatus = false;
unsigned long previousMillis = 0;
bool displayState = false;  // False means displaying temperature/humidity, true means displaying status
int waterThreshold = 500;   // Threshold for the water level sensor

void setup() {
  Serial.begin(9600);
  Serial.println("System Initializing...");

  dht.begin();
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Pin modes for relays
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);

  // Set relays to default states (OFF)
  digitalWrite(FAN_RELAY_PIN, HIGH);    // Fan OFF (active-low)
  digitalWrite(HUMIDIFIER_PIN, LOW);    // Humidifier OFF (active-low)
  digitalWrite(PUMP_RELAY_PIN, HIGH);   // Pump OFF (active-low)

  Serial.println("Setup Complete.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Update the sensor readings every 2 seconds
  if (currentMillis - previousMillis >= 2000) {
    previousMillis = currentMillis;

    // Reading temperature and humidity from the DHT11 sensor
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature(); // Read temperature as Celsius

    // Check if any reads failed
    if (isnan(humidity) || isnan(temperatureC)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Print the results to the Serial Monitor
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperatureC);
    Serial.println("°C");

    // Control the fan based on temperature and humidity
    if (temperatureC > 25 && humidity > 60) {
      digitalWrite(FAN_RELAY_PIN, LOW);  // Turn the fan on (active-low)
      fanStatus = true;
      Serial.println("Fan ON");
    } else {
      digitalWrite(FAN_RELAY_PIN, HIGH); // Turn the fan off (active-low)
      fanStatus = false;
      Serial.println("Fan OFF");
    }

    // Control the humidifier based on temperature and humidity
    if (temperatureC > 25 && humidity > 60) {
      digitalWrite(HUMIDIFIER_PIN, HIGH);  // Turn the humidifier on (active-low)
      humidifierStatus = true;
      Serial.println("Humidifier ON");
    } else {
      digitalWrite(HUMIDIFIER_PIN, LOW);   // Turn the humidifier off (active-low)
      humidifierStatus = false;
      Serial.println("Humidifier OFF");
    }

    // Reading the water level sensor
    int waterLevel = analogRead(WATER_SENSOR_PIN);
    Serial.print("Water Level Reading: ");
    Serial.println(waterLevel);

    // Ensure valid sensor readings
    if (waterLevel == 0 || waterLevel == 1023) {
      Serial.println("Sensor error or disconnected! Check connections.");
    }

    // Control the water pump based on the water level
    if (waterLevel < waterThreshold && waterLevel > 0) {
      digitalWrite(PUMP_RELAY_PIN, LOW);  // Turn ON the water pump (active-low)
      pumpStatus = true;
      Serial.println("Pump ON");
    } else if (waterLevel >= waterThreshold) {
      digitalWrite(PUMP_RELAY_PIN, HIGH); // Turn OFF the water pump (active-low)
      pumpStatus = false;
      Serial.println("Pump OFF");
    }

    // Toggle between displaying temperature/humidity and status every 2 seconds
    if (displayState) {
      // Display temperature and humidity on the LCD
      lcd.clear();
      lcd.setCursor(0, 0);   // Set cursor to first line
      lcd.print("Temp: ");
      lcd.print(temperatureC);
      lcd.print("C");

      lcd.setCursor(0, 1);   // Set cursor to second line
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
    } else {
      // Display fan, humidifier, and pump status on the LCD
      lcd.clear();
      lcd.setCursor(0, 0);   // Set cu`rsor to first line
      lcd.print("Fan: ");
      lcd.print(fanStatus ? "ON " : "OFF");

      lcd.setCursor(0, 1);   // Set cursor to second line
      lcd.print("Humidifier: ");
      lcd.print(humidifierStatus ? "ON " : "OFF");
      lcd.print(" Pump: ");
      lcd.print(pumpStatus ? "ON" : "OFF");
    }
    
    displayState = !displayState;  // Toggle display state
  }
}
