/*
Node description:
UDP server that sends data from the IMU to the Lifecyclenode receiver over WiFi.
*/ 

/*
--Software changes:
one line per change 
(1) created 03.11.2025: developer-Rik van Velzen 
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFiUdp.h>

// Wi-Fi credentials
const char* ssid = "IMU";
const char* password = "IMUsensor1234";

// UDP settings
const char* host_ip = "10.42.0.1"; // ROS2 PC IP

const int host_port = 5005;
WiFiUDP udp;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("Failed to find BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 initialized");
}

void loop() {
  sensors_event_t angVelocityData, linearAccelData;

  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Extract values
  float gx = angVelocityData.gyro.x;
  float gy = angVelocityData.gyro.y;
  float gz = angVelocityData.gyro.z;
  float ax = linearAccelData.acceleration.x;
  float ay = linearAccelData.acceleration.y;
  float az = linearAccelData.acceleration.z;

  // Print to Serial Monitor
  Serial.print("Gyroscope (rad/s): ");
  Serial.printf("X: %.3f  Y: %.3f  Z: %.3f | ", gx, gy, gz);
  Serial.print("Linear Accel (m/s²): ");
  Serial.printf("X: %.3f  Y: %.3f  Z: %.3f\n", ax, ay, az);

  // Format as CSV: gx,gy,gz,ax,ay,az
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
           gx, gy, gz, ax, ay, az);

  // Send via UDP
  udp.beginPacket(host_ip, host_port);
  udp.write((uint8_t*)buffer, strlen(buffer));  // <-- cast to uint8_t*
  udp.endPacket();

  delay(200); // 10 Hz
}
