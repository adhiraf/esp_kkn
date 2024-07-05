#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>

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
const char* mqtt_server = "broker.emqx.io";
WiFiClient espClient;
PubSubClient client(espClient);

// Constants
const int riverbedHeight = 200; // Example riverbed height in cm

// Timer variables
unsigned long motionDetectedTime = 0;
bool ledActive = false;
unsigned long lastPublishTime = 0; // Last time MQTT message was published
const unsigned long publishInterval = 5000; // 5 seconds

// Variables to hold TMA and hazard level
int lastTMA = 0;
int lastPumpStatus = 0;
int lastHazardLevel = 1;

// Variables for ultrasonic sensor filtering
const int numReadings = 10; // Using 10 measurements for moving average
long readings[numReadings];
int readIndex = 0;
long total = 0;
long average = 0;

// Function prototypes
void setup_wifi();
bool reconnect();
void sendMQTTData(int tma, int pumpStatus, int hazardLevel);
long getUltrasonicReading();
long correctReading(long distance);
void initSensorReadings();
void updateLCD(int TMA, int hazardLevel);

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

  // Initialize sensor readings
  initSensorReadings();

  // Connect to WiFi
  setup_wifi();

  // Setup MQTT
  client.setServer(mqtt_server, 1883);

  delay(100);
}

void loop() {
  unsigned long currentTime = millis();

  // Handle motion detection and LED activation
  if (mpu.getMotionInterruptStatus()) {
    // Motion detected
    motionDetectedTime = currentTime; // Update the last motion detected time
    ledActive = true;

    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print out the values
    Serial.println("Detect Motion!");
  }

  if (ledActive && (currentTime - motionDetectedTime < 5000)) { // 5 seconds
    digitalWrite(ledPin, HIGH);
  } else {
    ledActive = false;
    digitalWrite(ledPin, LOW);
  }

  int pumpStatus = ledActive ? 1 : 0;

  // Measure distance using ultrasonic sensor with moving average
  long averageDistance = getUltrasonicReading();

  // Calculate TMA
  int TMA = riverbedHeight - averageDistance;
  lastTMA = TMA; // Store the last TMA for MQTT publishing

  // Print debug information for average distance and TMA
  Serial.print("Average distance: ");
  Serial.print(averageDistance);
  Serial.print(" cm, TMA: ");
  Serial.print(TMA);
  Serial.println(" cm");

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
  lastHazardLevel = hazardLevel; // Store the last hazard level for MQTT publishing

  // Update LCD
  updateLCD(lastTMA, lastHazardLevel);

  // Handle MQTT connection and publishing
  if (currentTime - lastPublishTime >= publishInterval) {
    sendMQTTData(lastTMA, pumpStatus, lastHazardLevel);
    lastPublishTime = currentTime;
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  delay(1000); // Delay for 1 second
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

bool reconnect() {
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect("ESP32Client")) {
    Serial.println("connected");
    // Once connected, publish an announcement
    client.publish("KKNSTMKG15/status", "Connected");
    return true;
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 3 seconds");
    return false;
  }
}

void sendMQTTData(int tma, int pumpStatus, int hazardLevel) {
  // Create JSON object with StaticJsonDocument from ArduinoJson library
  StaticJsonDocument<100> doc; // Adjust the size as needed

  // Fill data into the JSON document
  doc["TMA"] = tma;
  doc["PumpStatus"] = pumpStatus;
  doc["HazardLevel"] = hazardLevel;

  // Serialize JSON into a string
  char jsonBuffer[100]; // Adjust the buffer size as needed
  serializeJson(doc, jsonBuffer);

  // Send message to the MQTT broker
  Serial.println("Publishing data!");
  client.publish("KKNSTMKG15/parameter", jsonBuffer);
  if (!client.publish("KKNSTMKG15/parameter", jsonBuffer)) {
    Serial.println("Failed to publish message");
  }
}

long getUltrasonicReading() {
  long sum = 0;
  for (int i = 0; i < numReadings; i++) {
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2; // One-way distance

    sum += distance;

    delay(50); // Small delay between readings
  }

  // Calculate the average distance
  long averageDistance = sum / numReadings;

  // Apply correction formula
  long correctedDistance = correctReading(averageDistance);

  // Print debug information for average ultrasonic sensor reading
  Serial.print("Corrected average distance: ");
  Serial.print(correctedDistance);
  Serial.println(" cm");

  return correctedDistance;
}

long correctReading(long distance) {
  double x = distance;
  double y = 0.0007 * x * x + 0.0748 * x - 3.1813;
  return distance + y;
}

void initSensorReadings() {
  // Initialize readings array with initial distance measurements
  for (int i = 0; i < numReadings; i++) {
    readings[i] = getUltrasonicReading();
    total += readings[i];
    delay(50); // Small delay between readings
  }
  average = total / numReadings;
}

void updateLCD(int TMA, int hazardLevel) {
  lcd.clear();
  delay(10);

  // Row 1: "Tinggi Muka Air" centered
  lcd.setCursor((20 - strlen("Tinggi Muka Air")) / 2, 0);
  lcd.print("Tinggi Muka Air");

  // Row 2: Distance with " cm" centered
  String distanceStr = String(TMA) + " cm";
  lcd.setCursor((20 - distanceStr.length()) / 2, 1);
  lcd.print(distanceStr);

  // Row 3: Hazard level centered
  String hazardLevelStr;
  switch (hazardLevel) {
    case 4:
      hazardLevelStr = "BAHAYA";
      break;
    case 3:
      hazardLevelStr = "SIAGA";
      break;
    case 2:
      hazardLevelStr = "WASPADA";
      break;
    case 1:
    default:
      hazardLevelStr = "NORMAL";
      break;
  }
  lcd.setCursor((20 - hazardLevelStr.length()) / 2, 2);
  lcd.print(hazardLevelStr);

  // Row 4: "KKN STMKG Unit 15" centered
  lcd.setCursor((20 - strlen("KKN STMKG Unit 15")) / 2, 3);
  lcd.print("KKN STMKG Unit 15");
}
