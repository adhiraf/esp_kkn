#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Pins for ultrasonic sensor
const int trigPin = 25;
const int echoPin = 26;

// LED pin (previously relay pin)
const int ledPin = 5;

// Create MPU6050 instance
Adafruit_MPU6050 mpu;

// Create LCD instance with I2C address 0x27 and 20x4 display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WiFi credentials
const char* ssid = "MeteoJuanda";
const char* password = "hiluxpertek"; // replace with your actual password

// MQTT broker details
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT topics
const char* mainTopic = "KKNSTMKG15/parameter";

// Constants
const int riverbedHeight = 200; // Example riverbed height in cm

// Timer variables
unsigned long motionDetectedTime = 0;
bool ledActive = false;

// Function prototypes
void setup_wifi();
void reconnect();
void sendMQTTData(int tma, int pumpStatus, int hazardLevel);

void setup(void) {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Initialize LCD
  lcd.init();        // Correct method to initialize the LCD
  lcd.backlight();   // Turn on the backlight

  // Setup pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Connect to WiFi
  setup_wifi();

  // Setup MQTT
  client.setServer(mqtt_server, 1883);

  delay(100);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (mpu.getMotionInterruptStatus()) {
    // Motion detected
    motionDetectedTime = millis(); // Update the last motion detected time
    ledActive = true;

    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print out the values
    Serial.println("Detect Motion!");
  }

  // Activate relay for 1 minute if motion was detected
  if (ledActive) {
    if (millis() - motionDetectedTime < 5000) { // 5 seconds
      digitalWrite(ledPin, HIGH);
    } else {
      ledActive = false;
      digitalWrite(ledPin, LOW);
    }
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Measure distance using ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Calculate TMA
  int TMA = riverbedHeight - distance;

  // Determine hazard level
  int hazardLevel;
  String hazardLevelStr;
  if (TMA >= 150) {
    hazardLevel = 4; // BAHAYA
    hazardLevelStr = "BAHAYA";
  } else if (TMA >= 120) {
    hazardLevel = 3; // SIAGA
    hazardLevelStr = "SIAGA";
  } else if (TMA >= 100) {
    hazardLevel = 2; // WASPADA
    hazardLevelStr = "WASPADA";
  } else {
    hazardLevel = 1; // NORMAL
    hazardLevelStr = "NORMAL";
  }

  // Send data to MQTT broker
  int pumpStatus = ledActive ? 1 : 0;
  sendMQTTData(TMA, pumpStatus, hazardLevel);
  
  // Row 1: "Tinggi Muka Air" centered
  lcd.setCursor((20 - strlen("Tinggi Muka Air")) / 2, 0);
  lcd.print("Tinggi Muka Air");

  // Row 2: Distance with " cm" centered
  String distanceStr = String(TMA) + " cm";
  lcd.setCursor((20 - distanceStr.length()) / 2, 1);
  lcd.print(distanceStr);

  // Row 3: Hazard level centered
  lcd.setCursor((20 - hazardLevelStr.length()) / 2, 2);
  lcd.print(hazardLevelStr);

  // Row 4: "KKN STMKG Unit 15" centered
  lcd.setCursor((20 - strlen("KKN STMKG Unit 15")) / 2, 3);
  lcd.print("KKN STMKG Unit 15");

  // Print distance and hazard level to Serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, TMA: ");
  Serial.print(TMA);
  Serial.print(" cm, Hazard Level: ");
  Serial.println(hazardLevelStr);

  delay(1000);  // Delay to avoid flickering
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  static bool firstConnect = true;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      if (firstConnect) {
        // Publish an announcement once
        client.publish("KKNSTMKG15/status", "Connected");
        firstConnect = false;
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendMQTTData(int tma, int pumpStatus, int hazardLevel) {
  String payload = "{";
  payload += "\"TMA\":" + String(tma) + ",";
  payload += "\"PumpStatus\":" + String(pumpStatus) + ",";
  payload += "\"HazardLevel\":" + String(hazardLevel);
  payload += "}";
  Serial.print("Publishing message: ");
  Serial.println(payload);
  client.publish(mainTopic, payload.c_str());
}
