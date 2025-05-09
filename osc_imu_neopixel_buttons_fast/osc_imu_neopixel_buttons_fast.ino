// This is the firmware for the IMU BNO 055 from Adafruit sensor, plus 2 Neopixels, and 2 buttons.
// Coded by Joaku de Sotavento
// 2025/10/16
// V.1.0

/*
Units from the BNO055 Sensor

VECTOR_ACCELEROMETER (values in m/s^2) (100Hz) Range: -39.2 m/s² to +39.2 m/s²
VECTOR_GYROSCOPE (Angular velocity) (values in rps, radians per second) (100Hz) Range: -34.91 rad/s to +34.91 rad/s
VECTOR_MAGNETOMETER (values in uT, micro Teslas) (20Hz) Range: -1300 µT to +1300 µT (X, Y axes), Z-axis range: -2500 µT to +2500 µT
getTemp (Temperature in Celsius degrees) (1Hz)
VECTOR_LINEARACCEL (values in m/s^2) (100Hz) Range: -39.2 m/s² to +39.2 m/s²
getQuat(void) (Quaternions, 100Hz) Range: -1 to 1
VECTOR_EULER (Euler angles, degrees from 0 to 359) (100Hz) Range: 0° to 359°
VECTOR_GRAVITY (Gravitational acceleration values in m/s²) (100Hz) -39.2 m/s² to +39.2 m/s²
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>

// Network and OSC configuration
char ssid[] = "Invisible";           
char pass[] = "Invisible";           
IPAddress ip(192, 168, 1, 11);       
IPAddress outIp(192, 168, 1, 100);   
const unsigned int outPort = 12000;  
WiFiUDP Udp;                         

// Sensor configuration
Adafruit_BNO055 bno = Adafruit_BNO055(55);  

// Buttons configuration
const int buttonPin1 = 6;
const int buttonPin2 = 7;

// Neopixel configuration
#define PIN 4        
#define NUMPIXELS 2  
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  
const int neoP1 = 0;
const int neoP2 = 1;

// LED for WiFi status
const int wifiLed = 10;
const int pwmFreq = 5000;     
const int pwmResolution = 8;  

// Timers for controlling update frequency
unsigned long lastUpdate = 0;
unsigned long lastMagUpdate = 0;
unsigned long lastTempUpdate = 0;

// Set the sample rates (in milliseconds)
const int accelGyroInterval = 10;  // 100Hz = 10ms interval
const int magInterval = 50;        // 20Hz = 50ms interval
const int tempInterval = 1000;     // 1Hz = 1000ms interval
const int mainInterval = 5;        // Update every 5 ms

// Variables to store less frequently updated data
float magX = 0, magY = 0, magZ = 0;
int8_t boardTemp = 0;

void setup() {
  Serial.begin(115200);  
  Serial.println("IMU + Beetle ESP32 C3 and buttons");

  // Configure buttons
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  // Configure WiFi and connect
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected to LA INVISIBLE");

  // LED indication for WiFi connection
  ledcAttach(wifiLed, pwmFreq, pwmResolution);  
  ledcWrite(wifiLed, 500);
  Udp.begin(outPort);  

  // Initialize BNO055 sensor
  if (!bno.begin()) {
    Serial.print("No BNO055 detected!");
    while (1);
  }
  delay(1000);  

  // Initialize Neopixels
  pixels.begin();
  pixels.show();  
}

void loop() {
  unsigned long currentTime = millis();

  // Read the button states
  bool button1State = readButton(buttonPin1);
  bool button2State = readButton(buttonPin2);

  // Neopixel control logic
  int redBrightness = map(2300, 2200, 2500, 0, 255);
  int blueBrightness = map(2400, 2200, 2500, 0, 255);
  setNeopixelColor(neoP1, redBrightness, 0, blueBrightness);
  setNeopixelColor(neoP2, blueBrightness, 50, 10);

  // Main update every 5ms
  if (currentTime - lastUpdate >= mainInterval) {
    sensors_event_t orientationData, angVelocityData, linearAccelData, accelerometerData, gravityData;
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    // Update magnetometer every 50ms
    if (currentTime - lastMagUpdate >= magInterval) {
      sensors_event_t magnetometerData;
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      magX = magnetometerData.magnetic.x;
      magY = magnetometerData.magnetic.y;
      magZ = magnetometerData.magnetic.z;
      lastMagUpdate = currentTime;
    }

    // Update temperature every 1000ms
    if (currentTime - lastTempUpdate >= tempInterval) {
      boardTemp = bno.getTemp();
      lastTempUpdate = currentTime;
    }

    // Send OSC message with all data in the correct order
    sendIMUData(&accelerometerData, &angVelocityData, &linearAccelData, &orientationData, &gravityData, button1State, button2State);
    lastUpdate = currentTime;
  }
}

// Send all IMU data and button states in one OSC message with the correct order
void sendIMUData(sensors_event_t* accelerometerData, sensors_event_t* angVelocityData, sensors_event_t* linearAccelData, sensors_event_t* orientationData, sensors_event_t* gravityData, bool button1State, bool button2State) {
  OSCMessage msg("/imuINV");

  // 1. Add accelerometer data
  msg.add(accelerometerData->acceleration.x).add(accelerometerData->acceleration.y).add(accelerometerData->acceleration.z);

  // 2. Add gyroscope data
  msg.add(angVelocityData->gyro.x).add(angVelocityData->gyro.y).add(angVelocityData->gyro.z);

  // 3. Add magnetometer data (updated every 50ms)
  msg.add(magX).add(magY).add(magZ);

  // 4. Add temperature data (updated every 1000ms)
  msg.add((int32_t)boardTemp);

  // 5. Add linear acceleration data
  msg.add(linearAccelData->acceleration.x).add(linearAccelData->acceleration.y).add(linearAccelData->acceleration.z);
  
  // 6. Add quaternion data
  imu::Quaternion quat = bno.getQuat();
  msg.add((float)quat.w()).add((float)quat.x()).add((float)quat.y()).add((float)quat.z());

  // 7. Add orientation data
  msg.add(orientationData->orientation.x).add(orientationData->orientation.y).add(orientationData->orientation.z);

  // 8. Add gravity acceleration data
  msg.add(gravityData->acceleration.x).add(gravityData->acceleration.y).add(gravityData->acceleration.z);

  // 9. Add button states
  msg.add((bool)(button1State ? 1 : 0)).add((bool)(button2State ? 1 : 0));

  // Send OSC message
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);   
  Udp.endPacket();  
  msg.empty();     
}

bool readButton(int pin) {
  return digitalRead(pin);
}

void setNeopixelColor(int neoPixel, int red, int green, int blue) {
  pixels.setPixelColor(neoPixel, pixels.Color(red, green, blue));
  pixels.show();
}

