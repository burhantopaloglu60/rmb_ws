#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() 
{
  Serial.begin(9600);

  bno.begin();
  bno.setExtCrystalUse(true);
}
//output:
//ms since launch           (timestamp)
//accelEvent.acceleration.x (linear acceleration x)
//accelEvent.acceleration.y (linear acceleration y)
//accelEvent.acceleration.y (linear acceleration z)
//gyroEvent.gyro.z          (angular velocity z)
void loop() {
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;

  // Get linear acceleration and gyroscope data
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Print timestamp (ms since start)
  Serial.print("Timestamp: ");
  Serial.print(accelEvent.timestamp);
  Serial.print(" ms");

  // Print x, y, z acceleration
  Serial.print(" | Accel (m/s²): ");
  Serial.print(accelEvent.acceleration.x); Serial.print(", ");
  Serial.print(accelEvent.acceleration.y); Serial.print(", ");
  Serial.print(accelEvent.acceleration.z);

  // Print rotation acceleration (angular velocity?)
  Serial.print(" | Rotation Accel (rad/s): ");
  Serial.println(gyroEvent.gyro.z);

  delay(200);
}