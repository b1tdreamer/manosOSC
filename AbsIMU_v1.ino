#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

// IMU
Adafruit_BNO055 imuSensor;
String imuName("imu/1");
float gyro[] = {0.0, 0.0, 0.0};
float accel[] = {0.0, 0.0, 0.0};
float euler[] = {0.0, 0.0, 0.0};
float orient[] = {0.0, 0.0, 0.0, 0.0};
void setupIMUSensor();
void updateIMUSensor();
void printImuCalibration();
unsigned long imuSendInterval = 20; // in milliseconds
unsigned long previousIMUSendTime = 0; // in milliseconds

// Wifi
int status = WL_IDLE_STATUS;
//char ssid[] = "TP-LINK_167A";
//char pass[] = "22551733";
char ssid[] = "ASUS_sensor";
char pass[] = "Ff1235813";
//char ssid[] = "ASUS";
//char pass[] = "123456789";
//char ssid[] = "ASUS2";
//char pass[] = "Ff1235813";
int locPort = 2390;
int remPort = 10001;
//int remPort = 9001;
IPAddress remIP(255, 255, 255, 255); //change this and set it to the Ip of the recievng computer ? (169,254,196,11)
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP Udp;

void setupWifi();
void printWifiStatus();
void updateWifi();

// Serial
void setupSerial();

void setup(void)
{  
  setupSerial();
  setupIMUSensor();
  setupWifi();
}

void loop() 
{
  updateIMUSensor();
  updateWifi();
}

void
setupIMUSensor()
{
  imuSensor = Adafruit_BNO055();

  /* Initialise the sensor */
  if(!imuSensor.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = imuSensor.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  imuSensor.setExtCrystalUse(true);
}

void 
setupSerial()
{
  Serial.begin(115200);

  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  
  */
}

void 
setupWifi()
{
  // use on board led to display wifi status
   pinMode(6, OUTPUT);
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    //status = WiFi.beginAP(ssid);

    digitalWrite(6, LOW); 
    delay(200);
    digitalWrite(6, HIGH);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(locPort);

  Serial.println("connected"); 
}

void 
printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void updateIMUSensor()
{
  //printImuCalibration();

  //imu::Vector<3> linAcc = imuSensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linAcc = imuSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  accel[0] = linAcc.x();
  accel[1] = linAcc.y();
  accel[2] = linAcc.z();
  
  imu::Vector<3> rotAcc = imuSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  gyro[0] = rotAcc.x();
  gyro[1] = rotAcc.y();
  gyro[2] = rotAcc.z();

  imu::Vector<3> eul = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);

  euler[0] = eul.x();
  euler[1] = eul.y();
  euler[2] = eul.z();
}

void
printImuCalibration()
{
  // get calibration info
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  imuSensor.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}


void updateWifi()
{  
  ///////////////////////
  // check wifi status //
  ///////////////////////
  
  status = WiFi.status();
  if(status != WL_CONNECTED) 
  {
    Udp.stop();
    setupWifi();
  }
  
  ///////////////////
  // send messages //
  ///////////////////

  unsigned long currentTime = millis();

  if(previousIMUSendTime + imuSendInterval <= currentTime)
  {
     previousIMUSendTime = currentTime;

     // send all data in xOsc Style

     String messageAddress = String("/") + imuName;
     OSCMessage msg(messageAddress.c_str());
  
      msg.add(euler[0]);
      msg.add(euler[1]);
      msg.add(euler[2]);

      msg.add(accel[0]);
      msg.add(accel[1]);
      msg.add(accel[2]);

      msg.add(gyro[0]);
      msg.add(gyro[1]);
      msg.add(gyro[2]);

      Udp.beginPacket(remIP, remPort);
      msg.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket(); // mark the end of the OSC Packet
      msg.empty(); // free space occupied by message
  }
}
