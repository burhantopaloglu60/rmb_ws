// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

<<<<<<< Updated upstream
Adafruit_BNO055 bno = Adafruit_BNO055(55);
long prevTimestamp = 0;
double prevAngVelocity = 0;

long timestamp = 0; 
double angVelocity = 0;
double orientationVect[3] = {0,0,0}; //x,y,z,

double angularAcceleration = 0.0f;

void setup() {
  Serial.begin(9600);

  bno.begin();
  bno.setExtCrystalUse(true);
}
// output to ros = (timestamp IRL + x orientation + y orientation + z orientation + angular acceleration from topdown)
// Angular accel (rad/s²), xyz (Quaternion)
void loop() {
  //sensors_event_t accelEvent; the accelerometer has no angular acceleration, not needed
  sensors_event_t gyroEvent;

  // Get linear acceleration and gyroscope data
  //bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  //throw everything in to variables!
  angVelocity = gyroEvent.gyro.z;
  orientationVect[0] = gyroEvent.orientation.x;
  orientationVect[1] = gyroEvent.orientation.y;
  orientationVect[2] = gyroEvent.orientation.z;
  timestamp = gyroEvent.timestamp;

  if(prevTimestamp > 0 )
  {
    // calculate angular acceleration and send it in ros message
    // angular acceleration = dω​ / dt 
    angularAcceleration = ( (angVelocity - prevAngVelocity) / ( (double)(timestamp - prevTimestamp)/1000) ); // we get dt in seconds and divide the delta velocity by it 
    Serial.print("orientation x: ");
    Serial.print(orientationVect[0]);
    Serial.print(" | orientation y: ");
    Serial.print(orientationVect[1]);
    Serial.print(" | orientation z: ");
    Serial.print(orientationVect[2]);
    Serial.print(" | rotation accel in rad/s²: ");
    Serial.println(angularAcceleration);

  }
  prevAngVelocity = angVelocity;
  prevTimestamp = timestamp; // save previous timestamp to get angular accel

  delay(200); // polling timer
}

=======
// Adafruit_BNO055 bno = Adafruit_BNO055(55);

// void setup() 
// {
//   Serial.begin(9600);

//   bno.begin();
//   bno.setExtCrystalUse(true);
// }
// //timestamp, x, y, z, rotation acceleration. or at least what I understood of it
// void loop() {
//   sensors_event_t accelEvent;
//   sensors_event_t gyroEvent;

//   // Get linear acceleration and gyroscope data
//   bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

//   // Print timestamp
//   Serial.print("Timestamp: ");
//   Serial.print(accelEvent.timestamp);
//   Serial.print(" ms");

//   // Print x, y, z acceleration
//   Serial.print(" | Accel (m/s²): ");
//   Serial.print(accelEvent.acceleration.x); Serial.print(", ");
//   Serial.print(accelEvent.acceleration.y); Serial.print(", ");
//   Serial.print(accelEvent.acceleration.z);

//   // Print rotation acceleration (angular velocity?)
//   Serial.print(" | Rotation Accel (rad/s): ");
//   Serial.print(gyroEvent.gyro.x); Serial.print(", ");
//   Serial.print(gyroEvent.gyro.y); Serial.print(", ");
//   Serial.println(gyroEvent.gyro.z);

//   delay(100);
// }
>>>>>>> Stashed changes
