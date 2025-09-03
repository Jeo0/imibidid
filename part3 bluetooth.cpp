#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "HardwareSerial.h"

// Set the LCD address and dimensions
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT11 sensor pin and type
#define DHT_PIN 4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Soil moisture sensor pin
#define MOISTURE_PIN 34

// Relay module pin
#define RELAY_PIN 2

// HC-05 Bluetooth serial communication
HardwareSerial SerialBT(2);

// Define a threshold for watering (adjust as needed, 0 = dry, 4095 = wet)
#define MOISTURE_THRESHOLD 1500

void setup() {
  Serial.begin(115200);
  SerialBT.begin(9600); // Bluetooth default baud rate

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize DHT sensor
  dht.begin();
  
  // Set relay pin as output and start off
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Assuming a LOW signal activates the relay

  lcd.setCursor(0, 0);
  lcd.print("Smart Garden");
  lcd.setCursor(0, 1);
  lcd.print("System Ready");
  delay(2000);
}

void loop() {
  // Read sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int moisture = analogRead(MOISTURE_PIN);

  // Check if sensor readings are valid
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    return;
  }

  // Display data on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C H:");
  lcd.print(humidity);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Moisture:");
  lcd.print(moisture);

  // Check if soil is dry and water the plant
  if (moisture < MOISTURE_THRESHOLD) {
    Serial.println("Soil is dry! Watering...");
    SerialBT.println("Soil is dry! Watering...");
    lcd.setCursor(0, 1);
    lcd.print("Watering...");
    digitalWrite(RELAY_PIN, LOW); // Turn on the pump
    delay(5000); // Run pump for 5 seconds
    digitalWrite(RELAY_PIN, HIGH); // Turn off the pump
    delay(2000); // Wait for moisture to be absorbed
  }

  // Handle Bluetooth commands
  if (SerialBT.available()) {
    char command = SerialBT.read();
    if (command == 'W') {
      Serial.println("Manual water command received!");
      SerialBT.println("Manual watering initiated.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Manual Water");
      digitalWrite(RELAY_PIN, LOW);
      delay(5000);
      digitalWrite(RELAY_PIN, HIGH);
    }
  }

  // Send sensor data to Bluetooth
  String dataString = "T:" + String(temperature) + ",H:" + String(humidity) + ",M:" + String(moisture);
  SerialBT.println(dataString);
  
  delay(5000); // Wait before the next reading cycle
}
